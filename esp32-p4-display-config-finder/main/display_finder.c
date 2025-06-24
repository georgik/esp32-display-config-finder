/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 *
 * Interactive shell for tuning ILI9881C 1280x800 panel at runtime.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_event.h"
#include "console_simple_init.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_ili9881c.h"
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_ldo_regulator.h"
#include "esp_timer.h"

#include "driver/ledc.h"
#include "esp_system.h"

static const char *TAG = "disp_shell";

static volatile bool panel_running = false;

// Handle for our panel_loop task
static TaskHandle_t panel_task_handle = NULL;

static SemaphoreHandle_t draw_done_sem = NULL;

// Callback invoked when a color transfer completes
static bool color_trans_done_cb(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(draw_done_sem, &xHigherPriorityTaskWoken);
    return true;
}

// Runtime-configurable panel parameters
static struct {
    int dpi_clock_mhz;
    int h_size, v_size;
    int hs_pw, hs_bp, hs_fp;
    int vs_pw, vs_bp, vs_fp;
    int lanes;
    int lane_rate_mbps;
    int ldo_vci_chan;
    int ldo_vci_mv;
    int ldo_iovcc_chan;
    int ldo_iovcc_mv;
    int use_dma2d;
    int disable_lp;
    char color_name[8];      // default draw color: red, green, blue, white, black
} panel_cfg = {
    .dpi_clock_mhz = 60,
    .h_size        = 800,
    .v_size        = 1280,
    .hs_pw         = 16,         // Match working ESP-BSP example
    .hs_bp         = 172,        // Match working ESP-BSP example (CRITICAL)
    .hs_fp         = 32,         // Match working ESP-BSP example
    .vs_pw         = 4,          // Match working ESP-BSP example
    .vs_bp         = 40,         // Match working ESP-BSP example
    .vs_fp         = 26,         // Match working ESP-BSP example
    .lanes         = 2, // P4: max 2 lanes
    .lane_rate_mbps = 600,       // Match working ESP-BSP example
    .ldo_vci_chan   = 3,    // default VCI LDO channel
    .ldo_vci_mv     = 2700, // default analog voltage (mV)
    .ldo_iovcc_chan = 2,    // default IOVCC LDO channel
    .ldo_iovcc_mv   = 1900, // default logic voltage (mV)
    .use_dma2d    = 1,
    .disable_lp   = 1,
    .color_name    = "blue",
};

typedef struct { const char *name; uint8_t r, g, b; } color_map_t;
static const color_map_t color_map[] = {
    { "red",   255,   0,   0 },
    { "green",   0, 255,   0 },
    { "blue",    0,   0, 255 },
    { "white", 255, 255, 255 },
    { "black",   0,   0,   0 },
};
static void get_rgb_for_color(const char *name, uint8_t *r, uint8_t *g, uint8_t *b) {

    for (size_t i = 0; i < sizeof(color_map)/sizeof(color_map[0]); ++i) {
        if (strcmp(name, color_map[i].name) == 0) {
            *r = color_map[i].r;
            *g = color_map[i].g;
            *b = color_map[i].b;
            return;
        }
    }
    *r = *g = *b = 0;
}

// LCD handles
static esp_lcd_dsi_bus_handle_t   dsi_bus    = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t ctrl_panel = NULL;
static esp_lcd_panel_handle_t data_panel = NULL;
static uint8_t *s_fb_buf = NULL;
static size_t s_fb_size = 0;

// Forward declarations
static void panel_loop(void *pvParameters);
static void cleanup_panel(void);
static int cmd_sweep(int argc, char** argv);
static int cmd_autotest(int argc, char** argv);
static int cmd_diagnose(int argc, char** argv);

// 'show' command: display current config
static int cmd_show(int argc, char** argv) {
    ESP_LOGI(TAG, "Panel config:");
    ESP_LOGI(TAG, "  dpi=%d", panel_cfg.dpi_clock_mhz);
    ESP_LOGI(TAG, "  hsize=%d", panel_cfg.h_size);
    ESP_LOGI(TAG, "  vsize=%d", panel_cfg.v_size);
    ESP_LOGI(TAG, "  hs_pw=%d hs_bp=%d hs_fp=%d", panel_cfg.hs_pw, panel_cfg.hs_bp, panel_cfg.hs_fp);
    ESP_LOGI(TAG, "  vs_pw=%d vs_bp=%d vs_fp=%d", panel_cfg.vs_pw, panel_cfg.vs_bp, panel_cfg.vs_fp);
    ESP_LOGI(TAG, "  lanes=%d", panel_cfg.lanes);
    ESP_LOGI(TAG, "  lane_rate=%d", panel_cfg.lane_rate_mbps);
    ESP_LOGI(TAG, "  vci_chan=%d", panel_cfg.ldo_vci_chan);
    ESP_LOGI(TAG, "  vci_mv=%d", panel_cfg.ldo_vci_mv);
    ESP_LOGI(TAG, "  iovcc_chan=%d", panel_cfg.ldo_iovcc_chan);
    ESP_LOGI(TAG, "  iovcc_mv=%d", panel_cfg.ldo_iovcc_mv);
    ESP_LOGI(TAG, "  use_dma2d=%d", panel_cfg.use_dma2d);
    ESP_LOGI(TAG, "  disable_lp=%d", panel_cfg.disable_lp);
    ESP_LOGI(TAG, "  color=%s", panel_cfg.color_name);
    return 0;
}

// 'set' command: modify a parameter
static int cmd_set(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: set <param> <value>\n");
        return 0;
    }
    const char *p = argv[1];
    int v = atoi(argv[2]);
    if      (strcmp(p, "dpi") == 0) panel_cfg.dpi_clock_mhz = v;
    else if (strcmp(p, "hsize")==0) panel_cfg.h_size        = v;
    else if (strcmp(p, "vsize")==0) panel_cfg.v_size        = v;
    else if (strcmp(p, "hs_pw")==0) panel_cfg.hs_pw         = v;
    else if (strcmp(p, "hs_bp")==0) panel_cfg.hs_bp         = v;
    else if (strcmp(p, "hs_fp")==0) panel_cfg.hs_fp         = v;
    else if (strcmp(p, "vs_pw")==0) panel_cfg.vs_pw         = v;
    else if (strcmp(p, "vs_bp")==0) panel_cfg.vs_bp         = v;
    else if (strcmp(p, "vs_fp")==0) panel_cfg.vs_fp         = v;
    else if (strcmp(p, "lanes")==0) panel_cfg.lanes         = v;
    else if (strcmp(p, "lane_rate") == 0) panel_cfg.lane_rate_mbps = v;
    else if (strcmp(p, "vci_chan")==0)   panel_cfg.ldo_vci_chan   = v;
    else if (strcmp(p, "vci_mv")==0)     panel_cfg.ldo_vci_mv     = v;
    else if (strcmp(p, "iovcc_chan")==0) panel_cfg.ldo_iovcc_chan = v;
    else if (strcmp(p, "iovcc_mv")==0)   panel_cfg.ldo_iovcc_mv   = v;
    else if (strcmp(p, "use_dma2d")==0)     panel_cfg.use_dma2d    = v;
    else if (strcmp(p, "disable_lp")==0)    panel_cfg.disable_lp   = v;
    else if (strcmp(p, "color") == 0) {
        strncpy(panel_cfg.color_name, argv[2], sizeof(panel_cfg.color_name) - 1);
        panel_cfg.color_name[sizeof(panel_cfg.color_name) - 1] = '\0';
    }
    else {
        printf("Unknown param '%s'\n", p);
        return 0;
    }
    // Special-case log for color, otherwise generic log
    if (strcmp(p, "color") == 0) {
        ESP_LOGI(TAG, "Set %s = %s", p, panel_cfg.color_name);
    } else {
        ESP_LOGI(TAG, "Set %s = %d", p, v);
    }
    //ESP_LOGI(TAG, "Set %s = %d", p, v); // old generic log removed/commented
    return 0;
}

// 'stop' command: stop continuous draw loop
static int cmd_stop(int argc, char** argv) {
    panel_running = false;
    ESP_LOGI(TAG, "Panel stop requested");
    // Wait for the panel_loop task to exit and clear its handle
    while (panel_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return 0;
}

// 'oneframe' command: initialize panel and push a single frame without continuous loop
static int cmd_oneframe(int argc, char** argv) {
    esp_err_t ret = ESP_OK;
    
    // Stop any running panel task first
    if (panel_running || panel_task_handle) {
        cmd_stop(0, NULL);
    }
    
    ESP_LOGI(TAG, "Single frame mode - initializing panel...");
    
    // Power up MIPI DSI PHY: analog (VCI) and logic (IOVCC) rails
    esp_ldo_channel_handle_t ldo_vci = NULL;
    esp_ldo_channel_config_t ldo_vci_cfg = {
        .chan_id    = panel_cfg.ldo_vci_chan,
        .voltage_mv = panel_cfg.ldo_vci_mv,
    };
    ret = esp_ldo_acquire_channel(&ldo_vci_cfg, &ldo_vci);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to acquire VCI LDO chan %d at %dmV: %s",
                 panel_cfg.ldo_vci_chan, panel_cfg.ldo_vci_mv, esp_err_to_name(ret));
    }

    esp_ldo_channel_handle_t ldo_iovcc = NULL;
    esp_ldo_channel_config_t ldo_iovcc_cfg = {
        .chan_id    = panel_cfg.ldo_iovcc_chan,
        .voltage_mv = panel_cfg.ldo_iovcc_mv,
    };
    ret = esp_ldo_acquire_channel(&ldo_iovcc_cfg, &ldo_iovcc);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to acquire IOVCC LDO chan %d at %dmV: %s",
                 panel_cfg.ldo_iovcc_chan, panel_cfg.ldo_iovcc_mv, esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Reconfiguring panel...");
    cleanup_panel();

    // Calculate lane bit rate based on pixel clock (MHz), bits per pixel (24 for RGB888), and number of lanes
    int default_rate = panel_cfg.dpi_clock_mhz * 24 / (panel_cfg.lanes * 2);
    int lane_rate_mbps = panel_cfg.lane_rate_mbps > 0 ? panel_cfg.lane_rate_mbps : default_rate;
    ESP_LOGI(TAG, "DSI lane bit rate: %d Mbps (default %d)", lane_rate_mbps, default_rate);

    // 1) DSI bus
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id             = 0,
        .num_data_lanes     = panel_cfg.lanes,     // number of DSI data lanes
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = lane_rate_mbps,
    };
    ret = esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DSI bus: %s", esp_err_to_name(ret));
        return 1;
    }
    ESP_LOGI(TAG, "DSI bus created successfully");

    // Create DBI IO for control commands
    esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    ret = esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DBI IO: %s", esp_err_to_name(ret));
        return 1;
    }
    ESP_LOGI(TAG, "DBI IO created");

    // DPI timing
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = panel_cfg.dpi_clock_mhz,
        .virtual_channel    = 0,
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .in_color_format    = LCD_COLOR_FMT_RGB888,
        .out_color_format   = LCD_COLOR_FMT_RGB888,
        .num_fbs            = 1,
        .video_timing = {
            .h_size            = panel_cfg.h_size,
            .v_size            = panel_cfg.v_size,
            .hsync_pulse_width = panel_cfg.hs_pw,
            .hsync_back_porch  = panel_cfg.hs_bp,
            .hsync_front_porch = panel_cfg.hs_fp,
            .vsync_pulse_width = panel_cfg.vs_pw,
            .vsync_back_porch  = panel_cfg.vs_bp,
            .vsync_front_porch = panel_cfg.vs_fp,
        },
        .flags.use_dma2d   = panel_cfg.use_dma2d,
        .flags.disable_lp   = panel_cfg.disable_lp,
    };

    // Vendor config with Yeebo-specific initialization
    ili9881c_vendor_config_t vendor_cfg = {
        .mipi_config = { .dsi_bus = dsi_bus, .dpi_config = &dpi_cfg, .lane_num = panel_cfg.lanes },
        .init_cmds = vendor_specific_init_yeebo,
        .init_cmds_size = vendor_specific_init_yeebo_size
    };

    // Install control panel driver
    esp_lcd_panel_dev_config_t ctrl_cfg = {
        .reset_gpio_num = GPIO_NUM_27, // no reset GPIO
        .rgb_ele_order  = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = 24,
        .vendor_config  = &vendor_cfg,  // reuse vendor_cfg
    };
    ESP_LOGI(TAG, "Creating ILI9881C control panel...");
    ret = esp_lcd_new_panel_ili9881c(io_handle, &ctrl_cfg, &ctrl_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Control panel creation failed: %s", esp_err_to_name(ret));
        return 1;
    }
    ESP_LOGI(TAG, "Control panel created");

    ESP_LOGI(TAG, "Resetting control panel...");
    ret = esp_lcd_panel_reset(ctrl_panel);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Control reset failed: %s", esp_err_to_name(ret)); return 1; }
    ESP_LOGI(TAG, "Init control panel...");
    ret = esp_lcd_panel_init(ctrl_panel);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Control init failed: %s", esp_err_to_name(ret)); return 1; }
    ESP_LOGI(TAG, "Turning on control panel display...");
    ret = esp_lcd_panel_disp_on_off(ctrl_panel, true);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Control display on failed: %s", esp_err_to_name(ret)); return 1; }

    // Create DPI data panel for pixel transfers
    ESP_LOGI(TAG, "Creating DPI data panel...");
    ret = esp_lcd_new_panel_dpi(dsi_bus, &dpi_cfg, &data_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DPI panel creation failed: %s", esp_err_to_name(ret));
        return 1;
    }
    ESP_LOGI(TAG, "DPI panel created");
    ESP_LOGI(TAG, "Init DPI panel...");
    ret = esp_lcd_panel_init(data_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DPI init failed: %s", esp_err_to_name(ret));
        return 1;
    }
    ESP_LOGI(TAG, "Turning on data panel display...");
    ret = esp_lcd_panel_disp_on_off(data_panel, true);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "DPI panel disp_on_off not supported; continuing without explicit display-on call");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DPI display on failed: %s", esp_err_to_name(ret));
    }

    // Create binary semaphore for draw completion
    draw_done_sem = xSemaphoreCreateBinary();
    if (!draw_done_sem) {
        ESP_LOGE(TAG, "Failed to create draw_done_sem");
    } else {
        esp_lcd_dpi_panel_event_callbacks_t cbs = {
            .on_color_trans_done = color_trans_done_cb,
        };
        ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(data_panel, &cbs, NULL));
    }

    // Use DPI panel's internal frame buffer
    void *fb_ptr = NULL;
    ESP_LOGI(TAG, "Getting internal frame buffer from DPI panel");
    ret = esp_lcd_dpi_panel_get_frame_buffer(data_panel, 1, &fb_ptr);
    if (ret != ESP_OK || !fb_ptr) {
        ESP_LOGE(TAG, "Failed to get DPI frame buffer: %s", esp_err_to_name(ret));
        cleanup_panel();
        return 1;
    }
    s_fb_buf = (uint8_t*)fb_ptr;
    s_fb_size = panel_cfg.h_size * panel_cfg.v_size * 3; // 3 bytes per pixel
    ESP_LOGI(TAG, "Using DPI internal frame buffer at %p, size %u bytes", s_fb_buf, (unsigned)s_fb_size);

    // Determine RGB values based on current color setting
    uint8_t r, g, b;
    get_rgb_for_color(panel_cfg.color_name, &r, &g, &b);
    ESP_LOGI(TAG, "Single frame color: %s (R:%d, G:%d, B:%d)", panel_cfg.color_name, r, g, b);

    // Fill the internal framebuffer with the selected color
    for (size_t i = 0; i < s_fb_size; i += 3) {
        s_fb_buf[i + 0] = b;  // Blue component
        s_fb_buf[i + 1] = g;  // Green component
        s_fb_buf[i + 2] = r;  // Red component
    }
    ESP_LOGI(TAG, "Frame buffer filled with color");
    
    // Push single frame to panel
    ESP_LOGI(TAG, "Pushing single frame to panel");
    esp_lcd_panel_draw_bitmap(data_panel, 0, 0, panel_cfg.h_size, panel_cfg.v_size, s_fb_buf);
    if (draw_done_sem) {
        xSemaphoreTake(draw_done_sem, portMAX_DELAY);
    }
    ESP_LOGI(TAG, "Single frame displayed successfully!");
    
    // Keep the display on but stop here (no continuous refresh)
    ESP_LOGI(TAG, "Single frame mode complete - panel initialized and single frame displayed");
    
    return 0;
}

// 'start' command: re-init panel with current settings and start continuous draw loop
static int cmd_start(int argc, char** argv) {
    if (panel_running || panel_task_handle) {
        cmd_stop(0, NULL);
    }
    BaseType_t res = xTaskCreate(
        panel_loop,
        "panel_loop",
        8192,
        NULL,
        tskIDLE_PRIORITY + 1,
        &panel_task_handle
    );
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to start panel task");
    }
    return 0;
}


// 'reboot' command: reboot the ESP chip
static int cmd_reboot(int argc, char** argv) {
    ESP_LOGI(TAG, "Rebooting system...");
    esp_restart();
    return 0;
}

static int cmd_peek(int argc, char** argv) {
    if (s_fb_buf == NULL) {
        ESP_LOGW(TAG, "Framebuffer not allocated yet");
        return 0;
    }
    size_t count = 16;
    if (argc >= 2) {
        count = atoi(argv[1]);
    }
    if (count > s_fb_size) {
        count = s_fb_size;
    }
    printf("FB[0..%u]:", (unsigned)count);
    for (size_t i = 0; i < count; i++) {
        printf(" %02X", s_fb_buf[i]);
    }
    printf("\n");
    return 0;
}

// 'diagnose' command: comprehensive framebuffer and DMA diagnostics
static int cmd_diagnose(int argc, char** argv) {
    if (s_fb_buf == NULL || data_panel == NULL) {
        ESP_LOGW(TAG, "Panel not initialized. Run 'oneframe' or 'start' first.");
        return 0;
    }
    
    ESP_LOGI(TAG, "\n=== FRAMEBUFFER & DMA DIAGNOSTICS ===");
    
    // 1. Basic framebuffer info
    ESP_LOGI(TAG, "Framebuffer address: %p", s_fb_buf);
    ESP_LOGI(TAG, "Framebuffer size: %u bytes (%dx%d x 3 bytes/pixel)", 
             (unsigned)s_fb_size, panel_cfg.h_size, panel_cfg.v_size);
    ESP_LOGI(TAG, "Expected size: %u bytes", (unsigned)(panel_cfg.h_size * panel_cfg.v_size * 3));
    
    // 2. Memory alignment check
    uintptr_t fb_addr = (uintptr_t)s_fb_buf;
    ESP_LOGI(TAG, "Memory alignment: %u bytes (should be 4+ for DMA)", 
             (unsigned)(fb_addr & 0x3F) == 0 ? 64 : (fb_addr & 0x3) == 0 ? 4 : 1);
    
    // 3. Memory accessibility test
    ESP_LOGI(TAG, "Testing memory accessibility...");
    volatile uint8_t test_val = 0xAA;
    volatile uint8_t orig_val = s_fb_buf[0];
    s_fb_buf[0] = test_val;
    if (s_fb_buf[0] == test_val) {
        ESP_LOGI(TAG, "✓ Framebuffer memory is writable");
        s_fb_buf[0] = orig_val; // Restore
    } else {
        ESP_LOGW(TAG, "✗ Framebuffer memory write test failed!");
    }
    
    // 4. Pattern integrity test
    ESP_LOGI(TAG, "Testing pattern integrity...");
    
    // Create a test pattern: alternating stripes
    const uint8_t pattern_colors[4][3] = {
        {0xFF, 0x00, 0x00}, // Red
        {0x00, 0xFF, 0x00}, // Green  
        {0x00, 0x00, 0xFF}, // Blue
        {0xFF, 0xFF, 0xFF}  // White
    };
    
    // Fill buffer with stripe pattern
    for (int y = 0; y < panel_cfg.v_size; y++) {
        int stripe = (y / 32) % 4; // 32-pixel high stripes
        for (int x = 0; x < panel_cfg.h_size; x++) {
            size_t idx = (y * panel_cfg.h_size + x) * 3;
            if (idx + 2 < s_fb_size) {
                s_fb_buf[idx + 0] = pattern_colors[stripe][2]; // B
                s_fb_buf[idx + 1] = pattern_colors[stripe][1]; // G  
                s_fb_buf[idx + 2] = pattern_colors[stripe][0]; // R
            }
        }
    }
    ESP_LOGI(TAG, "✓ Test pattern written to framebuffer");
    
    // 5. DMA transfer test with timing
    ESP_LOGI(TAG, "Testing DMA transfer timing...");
    
    uint32_t start_time = esp_timer_get_time();
    esp_err_t ret = esp_lcd_panel_draw_bitmap(data_panel, 0, 0, 
                                              panel_cfg.h_size, panel_cfg.v_size, s_fb_buf);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "✗ DMA transfer initiation failed: %s", esp_err_to_name(ret));
        return 1;
    }
    
    // Wait for DMA completion with timeout
    if (draw_done_sem) {
        if (xSemaphoreTake(draw_done_sem, pdMS_TO_TICKS(5000)) == pdTRUE) {
            uint32_t end_time = esp_timer_get_time();
            uint32_t transfer_time_us = end_time - start_time;
            ESP_LOGI(TAG, "✓ DMA transfer completed in %u microseconds", (unsigned)transfer_time_us);
            
            // Calculate expected transfer time and bandwidth
            float transfer_time_ms = transfer_time_us / 1000.0f;
            float bandwidth_mbps = (s_fb_size * 8.0f) / transfer_time_us; // Mbps
            ESP_LOGI(TAG, "Transfer rate: %.2f ms, %.2f Mbps", transfer_time_ms, bandwidth_mbps);
            
            // Bandwidth sanity check
            if (bandwidth_mbps < 100) {
                ESP_LOGW(TAG, "⚠ Low bandwidth detected - possible DMA bottleneck");
            } else if (bandwidth_mbps > 2000) {
                ESP_LOGW(TAG, "⚠ Unexpectedly high bandwidth - timing may be inaccurate");
            } else {
                ESP_LOGI(TAG, "✓ Bandwidth within normal range");
            }
        } else {
            ESP_LOGW(TAG, "✗ DMA transfer timeout after 5 seconds!");
        }
    } else {
        ESP_LOGW(TAG, "⚠ No semaphore available for DMA completion detection");
    }
    
    // 6. Pattern verification after display
    ESP_LOGI(TAG, "Verifying pattern integrity after DMA...");
    bool pattern_ok = true;
    int error_count = 0;
    
    // Check first few stripes
    for (int y = 0; y < 128 && y < panel_cfg.v_size; y += 32) {
        int expected_stripe = (y / 32) % 4;
        for (int x = 0; x < 10 && x < panel_cfg.h_size; x++) {
            size_t idx = (y * panel_cfg.h_size + x) * 3;
            if (idx + 2 < s_fb_size) {
                if (s_fb_buf[idx + 0] != pattern_colors[expected_stripe][2] ||
                    s_fb_buf[idx + 1] != pattern_colors[expected_stripe][1] ||
                    s_fb_buf[idx + 2] != pattern_colors[expected_stripe][0]) {
                    error_count++;
                    pattern_ok = false;
                    if (error_count <= 3) { // Limit error spam
                        ESP_LOGW(TAG, "Pattern mismatch at (%d,%d): got [%02X,%02X,%02X], expected [%02X,%02X,%02X]",
                                x, y, s_fb_buf[idx+0], s_fb_buf[idx+1], s_fb_buf[idx+2],
                                pattern_colors[expected_stripe][2], pattern_colors[expected_stripe][1], pattern_colors[expected_stripe][0]);
                    }
                }
            }
        }
    }
    
    if (pattern_ok) {
        ESP_LOGI(TAG, "✓ Pattern integrity verified");
    } else {
        ESP_LOGW(TAG, "✗ Pattern corruption detected (%d errors found)", error_count);
    }
    
    // 7. MIPI DSI lane utilization check
    ESP_LOGI(TAG, "\n=== MIPI DSI UTILIZATION ANALYSIS ===");
    int total_pixels = panel_cfg.h_size * panel_cfg.v_size;
    int total_bits = total_pixels * 24; // RGB888
    int refresh_rate_target = 60; // Hz
    int required_bandwidth_mbps = (total_bits * refresh_rate_target) / 1000000;
    
    ESP_LOGI(TAG, "Panel: %dx%d pixels, %d total bits per frame", 
             panel_cfg.h_size, panel_cfg.v_size, total_bits);
    ESP_LOGI(TAG, "Required bandwidth @ %dHz: %d Mbps", refresh_rate_target, required_bandwidth_mbps);
    ESP_LOGI(TAG, "Configured lane rate: %d Mbps x %d lanes = %d Mbps total", 
             panel_cfg.lane_rate_mbps, panel_cfg.lanes, panel_cfg.lane_rate_mbps * panel_cfg.lanes);
    
    float utilization = (float)required_bandwidth_mbps / (panel_cfg.lane_rate_mbps * panel_cfg.lanes) * 100.0f;
    ESP_LOGI(TAG, "Lane utilization: %.1f%%", utilization);
    
    if (utilization > 80.0f) {
        ESP_LOGW(TAG, "⚠ High lane utilization - consider increasing lane rate");
    } else if (utilization < 20.0f) {
        ESP_LOGW(TAG, "⚠ Very low utilization - lane rate may be too high");
    } else {
        ESP_LOGI(TAG, "✓ Lane utilization within acceptable range");
    }
    
    // 8. Memory heap info
    ESP_LOGI(TAG, "\n=== MEMORY STATUS ===");
    ESP_LOGI(TAG, "Free heap: %u bytes", (unsigned)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %u bytes", (unsigned)esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Free SPIRAM: %u bytes", (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    ESP_LOGI(TAG, "\n=== DIAGNOSTIC SUMMARY ===");
    ESP_LOGI(TAG, "Pattern display: Test stripes should be visible on panel");
    ESP_LOGI(TAG, "Check for: flickering, wrong colors, horizontal lines, or blank display");
    ESP_LOGI(TAG, "If issues persist, try adjusting: hs_bp, dpi, lane_rate parameters");
    
    return 0;
}

static void cleanup_panel(void) {
    if (data_panel)  { esp_lcd_panel_del(data_panel);  data_panel  = NULL; }
    if (ctrl_panel)  { esp_lcd_panel_del(ctrl_panel);  ctrl_panel  = NULL; }
    if (io_handle)   { esp_lcd_panel_io_del(io_handle); io_handle   = NULL; }
    if (dsi_bus)     { esp_lcd_del_dsi_bus(dsi_bus);    dsi_bus     = NULL; }
}

static void panel_loop(void *pvParameters) {
    esp_err_t ret = ESP_OK;
    panel_running = true;

    // Power up MIPI DSI PHY: analog (VCI) and logic (IOVCC) rails
    esp_ldo_channel_handle_t ldo_vci = NULL;
    esp_ldo_channel_config_t ldo_vci_cfg = {
        .chan_id    = panel_cfg.ldo_vci_chan,
        .voltage_mv = panel_cfg.ldo_vci_mv,
    };
    ret = esp_ldo_acquire_channel(&ldo_vci_cfg, &ldo_vci);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to acquire VCI LDO chan %d at %dmV: %s",
                 panel_cfg.ldo_vci_chan, panel_cfg.ldo_vci_mv, esp_err_to_name(ret));
    }


    esp_ldo_channel_handle_t ldo_iovcc = NULL;
    esp_ldo_channel_config_t ldo_iovcc_cfg = {
        .chan_id    = panel_cfg.ldo_iovcc_chan,
        .voltage_mv = panel_cfg.ldo_iovcc_mv,
    };
    ret = esp_ldo_acquire_channel(&ldo_iovcc_cfg, &ldo_iovcc);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to acquire IOVCC LDO chan %d at %dmV: %s",
                 panel_cfg.ldo_iovcc_chan, panel_cfg.ldo_iovcc_mv, esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Reconfiguring panel...");
    cleanup_panel();

    // Calculate lane bit rate based on pixel clock (MHz), bits per pixel (24 for RGB888), and number of lanes
    int default_rate = panel_cfg.dpi_clock_mhz * 24 / (panel_cfg.lanes * 2);
    int lane_rate_mbps = panel_cfg.lane_rate_mbps > 0 ? panel_cfg.lane_rate_mbps : default_rate;
    ESP_LOGI(TAG, "DSI lane bit rate: %d Mbps (default %d)", lane_rate_mbps, default_rate);

    // 1) DSI bus
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id             = 0,
        .num_data_lanes     = panel_cfg.lanes,     // number of DSI data lanes
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = lane_rate_mbps,
    };
    ret = esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DSI bus: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "DSI bus created successfully");

    // Create DBI IO for control commands
    esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    ret = esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DBI IO: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "DBI IO created");

    // DPI timing
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = panel_cfg.dpi_clock_mhz,
        .virtual_channel    = 0,
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .in_color_format    = LCD_COLOR_FMT_RGB888,
        .out_color_format   = LCD_COLOR_FMT_RGB888,
        .num_fbs            = 1,
        .video_timing = {
            .h_size            = panel_cfg.h_size,
            .v_size            = panel_cfg.v_size,
            .hsync_pulse_width = panel_cfg.hs_pw,
            .hsync_back_porch  = panel_cfg.hs_bp,
            .hsync_front_porch = panel_cfg.hs_fp,
            .vsync_pulse_width = panel_cfg.vs_pw,
            .vsync_back_porch  = panel_cfg.vs_bp,
            .vsync_front_porch = panel_cfg.vs_fp,
        },
        .flags.use_dma2d   = panel_cfg.use_dma2d,
        .flags.disable_lp   = panel_cfg.disable_lp,
    };

    // Vendor config with Yeebo-specific initialization
    ili9881c_vendor_config_t vendor_cfg = {
        .mipi_config = { .dsi_bus = dsi_bus, .dpi_config = &dpi_cfg, .lane_num = panel_cfg.lanes },
        .init_cmds = vendor_specific_init_yeebo,
        .init_cmds_size = vendor_specific_init_yeebo_size
    };

    // Install control panel driver
    esp_lcd_panel_dev_config_t ctrl_cfg = {
        .reset_gpio_num = GPIO_NUM_27, // no reset GPIO
        .rgb_ele_order  = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = 24,
        .vendor_config  = &vendor_cfg,  // reuse vendor_cfg
    };
    ESP_LOGI(TAG, "Creating ILI9881C control panel...");
    ret = esp_lcd_new_panel_ili9881c(io_handle, &ctrl_cfg, &ctrl_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Control panel creation failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Control panel created");

    ESP_LOGI(TAG, "Resetting control panel...");
    ret = esp_lcd_panel_reset(ctrl_panel);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Control reset failed: %s", esp_err_to_name(ret)); return; }
    ESP_LOGI(TAG, "Init control panel...");
    ret = esp_lcd_panel_init(ctrl_panel);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Control init failed: %s", esp_err_to_name(ret)); return; }
    ESP_LOGI(TAG, "Turning on control panel display...");
    ret = esp_lcd_panel_disp_on_off(ctrl_panel, true);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Control display on failed: %s", esp_err_to_name(ret)); return; }

    // Create DPI data panel for pixel transfers
    ESP_LOGI(TAG, "Creating DPI data panel...");
    ret = esp_lcd_new_panel_dpi(dsi_bus, &dpi_cfg, &data_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DPI panel creation failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "DPI panel created");
    ESP_LOGI(TAG, "Init DPI panel...");
    ret = esp_lcd_panel_init(data_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DPI init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Turning on data panel display...");
    ret = esp_lcd_panel_disp_on_off(data_panel, true);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "DPI panel disp_on_off not supported; continuing without explicit display-on call");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DPI display on failed: %s", esp_err_to_name(ret));
        // don't abort the task; just log and continue
    }

    // Create binary semaphore for draw completion
    draw_done_sem = xSemaphoreCreateBinary();
    if (!draw_done_sem) {
        ESP_LOGE(TAG, "Failed to create draw_done_sem");
    } else {
        esp_lcd_dpi_panel_event_callbacks_t cbs = {
            .on_color_trans_done = color_trans_done_cb,
        };
        ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(data_panel, &cbs, NULL));
    }

    // Use DPI panel's internal frame buffer instead of manual allocation
    void *fb_ptr = NULL;
    ESP_LOGI(TAG, "Getting internal frame buffer from DPI panel");
    ret = esp_lcd_dpi_panel_get_frame_buffer(data_panel, 1, &fb_ptr);
    if (ret != ESP_OK || !fb_ptr) {
        ESP_LOGE(TAG, "Failed to get DPI frame buffer: %s", esp_err_to_name(ret));
        cleanup_panel();
        return;
    }
    s_fb_buf = (uint8_t*)fb_ptr;
    s_fb_size = panel_cfg.h_size * panel_cfg.v_size * 3; // 3 bytes per pixel
    ESP_LOGI(TAG, "Using DPI internal frame buffer at %p, size %u bytes", s_fb_buf, (unsigned)s_fb_size);

    // Determine initial RGB values based on current color setting
    uint8_t r, g, b;
    get_rgb_for_color(panel_cfg.color_name, &r, &g, &b);

    // Fill the internal framebuffer once
    for (size_t i = 0; i < s_fb_size; i += 3) {
        s_fb_buf[i + 0] = b;
        s_fb_buf[i + 1] = g;
        s_fb_buf[i + 2] = r;
    }
    ESP_LOGI(TAG, "Initial internal buffer filled");
    // Push new buffer to panel
    ESP_LOGI(TAG, "Pushing initial buffer to panel");
    esp_lcd_panel_draw_bitmap(data_panel, 0, 0, panel_cfg.h_size, panel_cfg.v_size, s_fb_buf);
    if (draw_done_sem) {
        xSemaphoreTake(draw_done_sem, portMAX_DELAY);
    }

    // Automatic color rotation cycle
    const char* color_cycle[] = {"red", "green", "blue", "white", "black"};
    const int cycle_length = sizeof(color_cycle) / sizeof(color_cycle[0]);
    int color_index = 0;
    
    // Continuous update loop with automatic color cycling
    while (panel_running) {
        // Automatically cycle through colors
        const char* current_color = color_cycle[color_index];
        get_rgb_for_color(current_color, &r, &g, &b);
        
        ESP_LOGI(TAG, "Displaying color: %s (R:%d, G:%d, B:%d)", current_color, r, g, b);
        
        // Fill buffer with current color
        for (size_t i = 0; i < s_fb_size; i += 3) {
            s_fb_buf[i + 0] = b;  // Blue component
            s_fb_buf[i + 1] = g;  // Green component  
            s_fb_buf[i + 2] = r;  // Red component
        }
        
        // Push updated buffer to panel
        esp_lcd_panel_draw_bitmap(data_panel, 0, 0, panel_cfg.h_size, panel_cfg.v_size, s_fb_buf);
        if (draw_done_sem) {
            xSemaphoreTake(draw_done_sem, portMAX_DELAY);
        }
        
        // Move to next color in cycle
        color_index = (color_index + 1) % cycle_length;
        
        // Wait 1 second before next color
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //esp_lcd_panel_disp_on_off(data_panel, false);
    esp_lcd_panel_disp_on_off(ctrl_panel, false);
    // s_fb_buf now points to DPI internal buffer, don't free it manually
    s_fb_buf = NULL;
    cleanup_panel();
    if (draw_done_sem) {
        vSemaphoreDelete(draw_done_sem);
        draw_done_sem = NULL;
    }
    panel_running = false;
    panel_task_handle = NULL;
    vTaskDelete(NULL);
}

// 'sweep' command: iterate through a parameter range
// Usage: sweep <param> <start> <end> <step> [delay_ms]
static int cmd_sweep(int argc, char** argv) {
    if (argc < 5) {
        printf("Usage: sweep <param> <start> <end> <step> [delay_ms]\n");
        printf("Example: sweep hs_bp 100 200 10 2000\n");
        return 0;
    }
    
    const char *param = argv[1];
    int start = atoi(argv[2]);
    int end = atoi(argv[3]);
    int step = atoi(argv[4]);
    int delay_ms = argc >= 6 ? atoi(argv[5]) : 3000; // Default 3 seconds
    
    ESP_LOGI(TAG, "Starting parameter sweep: %s from %d to %d step %d (delay %dms)", 
             param, start, end, step, delay_ms);
    
    // Store original value to restore later
    int original_value = 0;
    if      (strcmp(param, "dpi") == 0) original_value = panel_cfg.dpi_clock_mhz;
    else if (strcmp(param, "hs_pw") == 0) original_value = panel_cfg.hs_pw;
    else if (strcmp(param, "hs_bp") == 0) original_value = panel_cfg.hs_bp;
    else if (strcmp(param, "hs_fp") == 0) original_value = panel_cfg.hs_fp;
    else if (strcmp(param, "vs_pw") == 0) original_value = panel_cfg.vs_pw;
    else if (strcmp(param, "vs_bp") == 0) original_value = panel_cfg.vs_bp;
    else if (strcmp(param, "vs_fp") == 0) original_value = panel_cfg.vs_fp;
    else if (strcmp(param, "lane_rate") == 0) original_value = panel_cfg.lane_rate_mbps;
    else {
        printf("Unsupported parameter for sweep: %s\n", param);
        return 0;
    }
    
    for (int value = start; value <= end; value += step) {
        ESP_LOGI(TAG, "\n=== SWEEP: %s = %d ===", param, value);
        
        // Set the parameter
        char val_str[16];
        snprintf(val_str, sizeof(val_str), "%d", value);
        char *set_argv[] = {"set", (char*)param, val_str};
        cmd_set(3, set_argv);
        
        // Display one frame
        ESP_LOGI(TAG, "Testing %s=%d...", param, value);
        cmd_oneframe(0, NULL);
        
        // Wait for user observation
        ESP_LOGI(TAG, "Displaying for %dms - observe the result", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    // Restore original value
    ESP_LOGI(TAG, "Sweep complete, restoring original %s=%d", param, original_value);
    char orig_str[16];
    snprintf(orig_str, sizeof(orig_str), "%d", original_value);
    char *restore_argv[] = {"set", (char*)param, orig_str};
    cmd_set(3, restore_argv);
    
    return 0;
}

// 'autotest' command: comprehensive automated testing
// Usage: autotest [quick|full]
static int cmd_autotest(int argc, char** argv) {
    const char *mode = argc >= 2 ? argv[1] : "quick";
    bool quick = strcmp(mode, "full") != 0;
    
    ESP_LOGI(TAG, "Starting automated test sequence (%s mode)", quick ? "quick" : "full");
    
    // Test sequence: colors first, then critical timing parameters
    const char* colors[] = {"red", "green", "blue", "white", "black"};
    const int num_colors = sizeof(colors) / sizeof(colors[0]);
    
    // 1. Color rotation test
    ESP_LOGI(TAG, "\n=== PHASE 1: COLOR ROTATION TEST ===");
    for (int i = 0; i < num_colors; i++) {
        ESP_LOGI(TAG, "Testing color: %s", colors[i]);
        char *color_argv[] = {"set", "color", (char*)colors[i]};
        cmd_set(3, color_argv);
        cmd_oneframe(0, NULL);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds per color
    }
    
    // 2. Critical timing parameter tests
    if (quick) {
        ESP_LOGI(TAG, "\n=== PHASE 2: QUICK TIMING TESTS ===");
        
        // Test horizontal back porch (most critical)
        ESP_LOGI(TAG, "Testing horizontal back porch variations...");
        int hs_bp_values[] = {150, 160, 172, 180, 190};
        for (int i = 0; i < 5; i++) {
            char val_str[16];
            snprintf(val_str, sizeof(val_str), "%d", hs_bp_values[i]);
            char *argv_set[] = {"set", "hs_bp", val_str};
            cmd_set(3, argv_set);
            ESP_LOGI(TAG, "Testing hs_bp=%d", hs_bp_values[i]);
            cmd_oneframe(0, NULL);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        
        // Test DPI clock
        ESP_LOGI(TAG, "Testing DPI clock variations...");
        int dpi_values[] = {50, 55, 60, 65, 70};
        for (int i = 0; i < 5; i++) {
            char val_str[16];
            snprintf(val_str, sizeof(val_str), "%d", dpi_values[i]);
            char *argv_set[] = {"set", "dpi", val_str};
            cmd_set(3, argv_set);
            ESP_LOGI(TAG, "Testing dpi=%d MHz", dpi_values[i]);
            cmd_oneframe(0, NULL);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    } else {
        ESP_LOGI(TAG, "\n=== PHASE 2: FULL TIMING PARAMETER SWEEP ===");
        
        // Comprehensive parameter sweep
        struct { const char *param; int start, end, step; } sweeps[] = {
            {"hs_bp", 140, 200, 10},
            {"hs_fp", 20, 50, 5},
            {"vs_bp", 30, 50, 5},
            {"vs_fp", 20, 35, 3},
            {"dpi", 50, 75, 5}
        };
        
        for (int i = 0; i < 5; i++) {
            ESP_LOGI(TAG, "Sweeping %s from %d to %d step %d", 
                     sweeps[i].param, sweeps[i].start, sweeps[i].end, sweeps[i].step);
            
            char start_str[16], end_str[16], step_str[16];
            snprintf(start_str, sizeof(start_str), "%d", sweeps[i].start);
            snprintf(end_str, sizeof(end_str), "%d", sweeps[i].end);
            snprintf(step_str, sizeof(step_str), "%d", sweeps[i].step);
            
            char *sweep_argv[] = {"sweep", (char*)sweeps[i].param, start_str, end_str, step_str, "2500"};
            cmd_sweep(6, sweep_argv);
        }
    }
    
    ESP_LOGI(TAG, "\n=== AUTOTEST COMPLETE ===");
    ESP_LOGI(TAG, "Restoring default configuration...");
    
    // Restore defaults
    char *restore_cmds[][3] = {
        {"set", "color", "blue"},
        {"set", "hs_bp", "172"},
        {"set", "dpi", "60"}
    };
    
    for (int i = 0; i < 3; i++) {
        cmd_set(3, restore_cmds[i]);
    }
    
    cmd_oneframe(0, NULL);
    ESP_LOGI(TAG, "Autotest sequence finished!");
    
    return 0;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting display shell");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(console_cmd_init());
    ESP_ERROR_CHECK(console_cmd_user_register("show", cmd_show));
    ESP_ERROR_CHECK(console_cmd_user_register("set", cmd_set));
    ESP_ERROR_CHECK(console_cmd_user_register("start", cmd_start));
    ESP_ERROR_CHECK(console_cmd_user_register("oneframe", cmd_oneframe));
    ESP_ERROR_CHECK(console_cmd_user_register("sweep", cmd_sweep));
    ESP_ERROR_CHECK(console_cmd_user_register("autotest", cmd_autotest));
    ESP_ERROR_CHECK(console_cmd_user_register("diagnose", cmd_diagnose));
    ESP_ERROR_CHECK(console_cmd_user_register("stop", cmd_stop));
    ESP_ERROR_CHECK(console_cmd_user_register("reboot", cmd_reboot));
    ESP_ERROR_CHECK(console_cmd_user_register("peek", cmd_peek));
    ESP_ERROR_CHECK(console_cmd_all_register());
    ESP_ERROR_CHECK(console_cmd_start());
    
    // Auto-execute "show" command on boot
    ESP_LOGI(TAG, "Auto-executing 'show' command:");
    cmd_show(0, NULL);
}
