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
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_ili9881c.h"
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_ldo_regulator.h"

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
    .hs_pw         = 16,
    .hs_bp         = 172,
    .hs_fp         = 32,
    .vs_pw         = 4,
    .vs_bp         = 40,
    .vs_fp         = 26,
    .lanes         = 2, // P4: max 2 lanes
    .lane_rate_mbps = 1000,
    .ldo_vci_chan   = 3,    // default VCI LDO channel
    .ldo_vci_mv     = 2700, // default analog voltage (mV)
    .ldo_iovcc_chan = 2,    // default IOVCC LDO channel
    .ldo_iovcc_mv   = 1900, // default logic voltage (mV)
    .use_dma2d    = 1,
    .disable_lp   = 0,
    .color_name    = "blue",
};

// LCD handles
static esp_lcd_dsi_bus_handle_t   dsi_bus    = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t ctrl_panel = NULL;
static esp_lcd_panel_handle_t data_panel = NULL;
static uint8_t *s_fb_buf = NULL;
static size_t s_fb_size = 0;

// Forward declarations
static void panel_loop(void *pvParameters);

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

// 'start' command: re-init panel with current settings and start continuous draw loop
static int cmd_start(int argc, char** argv) {
    if (panel_running || panel_task_handle) {
        ESP_LOGW(TAG, "Panel task already running, restarting...");
        cmd_stop(0, NULL); // stop if already running
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
    // teardown if already running
    if (ctrl_panel) { esp_lcd_panel_del(ctrl_panel); ctrl_panel = NULL; }
    if (data_panel) { esp_lcd_panel_del(data_panel); data_panel = NULL; }
    if (io_handle)  { esp_lcd_panel_io_del(io_handle); io_handle = NULL; }
    if (dsi_bus)    { esp_lcd_del_dsi_bus(dsi_bus);     dsi_bus = NULL; }

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

    // Vendor config
    ili9881c_vendor_config_t vendor_cfg = {
        .mipi_config = { .dsi_bus = dsi_bus, .dpi_config = &dpi_cfg, .lane_num = panel_cfg.lanes }
    };

    // Install control panel driver
    esp_lcd_panel_dev_config_t ctrl_cfg = {
        .reset_gpio_num = GPIO_NUM_NC, // no reset GPIO
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

    ESP_LOGI(TAG, "Filling screen with blue (full-frame buffer)");
    // Allocate full-frame buffer in PSRAM (32MB available)
    size_t total_px = panel_cfg.h_size * panel_cfg.v_size;
    size_t buf_sz = total_px * 3; // RGB888
    ESP_LOGI(TAG, "Allocating full-frame buffer of %u bytes", buf_sz);
    uint8_t *buf = heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    s_fb_buf = buf;
    s_fb_size = buf_sz;
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate full-frame buffer: %u bytes", buf_sz);
        return;
    }
    // Determine RGB values based on configured color
    uint8_t r = 0, g = 0, b = 0;
    if (strcmp(panel_cfg.color_name, "red") == 0) {
        r = 255; g = 0;   b = 0;
    } else if (strcmp(panel_cfg.color_name, "green") == 0) {
        r = 0;   g = 255; b = 0;
    } else if (strcmp(panel_cfg.color_name, "blue") == 0) {
        r = 0;   g = 0;   b = 255;
    } else if (strcmp(panel_cfg.color_name, "white") == 0) {
        r = 255; g = 255; b = 255;
    } else if (strcmp(panel_cfg.color_name, "black") == 0) {
        r = 0;   g = 0;   b = 0;
    } else {
        ESP_LOGW(TAG, "Unknown color '%s', defaulting to blue", panel_cfg.color_name);
        r = 0; g = 0; b = 255;
    }
    // Log setting pattern:
    printf("Using color: R=%d, G=%d, B=%d\n", r, g, b);

    // Fill buffer with the selected color
    for (size_t i = 0; i < buf_sz; i += 3) {
        buf[i + 0] = r;
        buf[i + 1] = g;
        buf[i + 2] = b;
    }
    ESP_LOGI(TAG, "Drawing full-frame bitmap...");

    // Continuous draw loop
    while (panel_running) {
        ret = esp_lcd_panel_draw_bitmap(data_panel, 0, 0, panel_cfg.h_size, panel_cfg.v_size, buf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Draw failed: %s", esp_err_to_name(ret));
            break;
        }
        // Wait for transfer completion
        if (draw_done_sem) {
            if (xSemaphoreTake(draw_done_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
                ESP_LOGW(TAG, "Draw completion timeout");
            }
        }
    }

    ESP_LOGI(TAG, "Cleaning up panels and DSI bus");
    free(buf);
    //esp_lcd_panel_disp_on_off(data_panel, false);
    esp_lcd_panel_disp_on_off(ctrl_panel, false);
    esp_lcd_panel_del(data_panel);
    data_panel = NULL;
    esp_lcd_panel_del(ctrl_panel);
    ctrl_panel = NULL;
    esp_lcd_panel_io_del(io_handle);
    io_handle = NULL;
    esp_lcd_del_dsi_bus(dsi_bus);
    dsi_bus = NULL;
    if (draw_done_sem) {
        vSemaphoreDelete(draw_done_sem);
        draw_done_sem = NULL;
    }
    panel_running = false;
    panel_task_handle = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting display shell");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(console_cmd_init());
    ESP_ERROR_CHECK(console_cmd_user_register("show", cmd_show));
    ESP_ERROR_CHECK(console_cmd_user_register("set", cmd_set));
    ESP_ERROR_CHECK(console_cmd_user_register("start", cmd_start));
    ESP_ERROR_CHECK(console_cmd_user_register("stop", cmd_stop));
    ESP_ERROR_CHECK(console_cmd_user_register("reboot", cmd_reboot));
    ESP_ERROR_CHECK(console_cmd_user_register("peek", cmd_peek));
    ESP_ERROR_CHECK(console_cmd_all_register());
    ESP_ERROR_CHECK(console_cmd_start());
}
