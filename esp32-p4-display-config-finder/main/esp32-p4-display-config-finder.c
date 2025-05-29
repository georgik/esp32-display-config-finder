/*
 * ESP-IDF app: Interactive shell for tuning ILI9881C 1280x800 panel parameters
 * Accepts commands over idf.py monitor via esp_console
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_lcd_dsi.h"
#include "esp_lcd_ili9881c.h"
#include "sdkconfig.h"

static const char* TAG = "disp_shell";

// Default panel parameters (modifiable via shell)
static struct {
    int dpi_clock_freq_mhz;
    int h_size;
    int v_size;
    int hsync_pw;
    int hsync_bp;
    int hsync_fp;
    int vsync_pw;
    int vsync_bp;
    int vsync_fp;
    int lane_num;
} panel_cfg = {
    .dpi_clock_freq_mhz = 71,
    .h_size             = 1280,
    .v_size             = 800,
    .hsync_pw           = 40,
    .hsync_bp           = 140,
    .hsync_fp           = 40,
    .vsync_pw           = 4,
    .vsync_bp           = 16,
    .vsync_fp           = 16,
    .lane_num           = 4,
};

// LCD handles
static esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel     = NULL;

// Forward declarations
static void init_panel_and_draw_blue(void);
static int cmd_show(int argc, char** argv);
static int cmd_set(int argc, char** argv);
static int cmd_init(int argc, char** argv);

// "show" command: print current config
static int cmd_show(int argc, char** argv) {
    ESP_LOGI(TAG, "Current panel parameters:");
    ESP_LOGI(TAG, " dpi_clock_freq_mhz: %d", panel_cfg.dpi_clock_freq_mhz);
    ESP_LOGI(TAG, " h_size: %d, v_size: %d", panel_cfg.h_size, panel_cfg.v_size);
    ESP_LOGI(TAG, " hsync: pw=%d bp=%d fp=%d", panel_cfg.hsync_pw, panel_cfg.hsync_bp, panel_cfg.hsync_fp);
    ESP_LOGI(TAG, " vsync: pw=%d bp=%d fp=%d", panel_cfg.vsync_pw, panel_cfg.vsync_bp, panel_cfg.vsync_fp);
    ESP_LOGI(TAG, " lane_num: %d", panel_cfg.lane_num);
    return 0;
}

// "set" command: set a parameter
static int cmd_set(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: set <param> <value>\n");
        return 0;
    }
    const char* param = argv[1];
    int value = atoi(argv[2]);
    if      (strcmp(param, "dpi_clock") == 0) panel_cfg.dpi_clock_freq_mhz = value;
    else if (strcmp(param, "h_size") == 0)    panel_cfg.h_size = value;
    else if (strcmp(param, "v_size") == 0)    panel_cfg.v_size = value;
    else if (strcmp(param, "hs_pw") == 0)     panel_cfg.hsync_pw = value;
    else if (strcmp(param, "hs_bp") == 0)     panel_cfg.hsync_bp = value;
    else if (strcmp(param, "hs_fp") == 0)     panel_cfg.hsync_fp = value;
    else if (strcmp(param, "vs_pw") == 0)     panel_cfg.vsync_pw = value;
    else if (strcmp(param, "vs_bp") == 0)     panel_cfg.vsync_bp = value;
    else if (strcmp(param, "vs_fp") == 0)     panel_cfg.vsync_fp = value;
    else if (strcmp(param, "lanes") == 0)     panel_cfg.lane_num = value;
    else {
        printf("Unknown param '%s'\n", param);
        return 0;
    }
    ESP_LOGI(TAG, "Set %s to %d", param, value);
    return 0;
}

// "init" command: initialize or reconfigure the panel and draw blue
static int cmd_init(int argc, char** argv) {
    init_panel_and_draw_blue();
    return 0;
}

// Register console commands
static void register_console_commands(void) {
    const esp_console_cmd_t show_cmd = {
        .command = "show",
        .help = "Show current panel configuration",
        .hint = NULL,
        .func = &cmd_show,
    };
    esp_console_cmd_register(&show_cmd);

    const esp_console_cmd_t set_cmd = {
        .command = "set",
        .help = "Set panel parameter: dpi_clock,h_size,v_size,hs_pw,hs_bp,hs_fp,vs_pw,vs_bp,vs_fp,lanes",
        .hint = "<param> <value>",
        .func = &cmd_set,
    };
    esp_console_cmd_register(&set_cmd);

    const esp_console_cmd_t init_cmd = {
        .command = "init",
        .help = "Initialize panel with current config and fill blue",
        .hint = NULL,
        .func = &cmd_init,
    };
    esp_console_cmd_register(&init_cmd);
}

// Console setup over UART0
static void init_console(void) {
    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Configure UART */
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize console */
    esp_console_config_t console_config = { .max_cmdline_args = 8, .max_cmdline_length = 128 }; 
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Enable linenoise for line editing */
    linenoiseSetMultiLine(1);
    linenoiseHistorySetMaxLen(100);
}

// Helper: init or re-init panel & draw solid blue
static void init_panel_and_draw_blue(void) {
    ESP_LOGI(TAG, "Initializing panel...");
    // If already initialized, delete old
    if (panel) {
        esp_lcd_panel_del(panel);
        panel = NULL;
    }
    if (io_handle) {
        esp_lcd_panel_io_del(io_handle);
        io_handle = NULL;
    }
    if (dsi_bus) {
        esp_lcd_del_dsi_bus(dsi_bus);
        dsi_bus = NULL;
    }

    // 1: create DSI bus
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id            = 0,
        .phy_clk_src       = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps= panel_cfg.dpi_clock_freq_mhz,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));

    // 2: create DBI-over-DSI I/O
    esp_lcd_dbi_io_config_t dbi_cfg = { .virtual_channel=0, .lcd_cmd_bits=8, .lcd_param_bits=8 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &io_handle));

    // 3: DPI timing
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .clk_src            = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = panel_cfg.dpi_clock_freq_mhz,
        .virtual_channel    = 0,
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .num_fbs            = 1,
        .video_timing = {
            .h_size            = panel_cfg.h_size,
            .v_size            = panel_cfg.v_size,
            .hsync_pulse_width = panel_cfg.hsync_pw,
            .hsync_back_porch  = panel_cfg.hsync_bp,
            .hsync_front_porch = panel_cfg.hsync_fp,
            .vsync_pulse_width = panel_cfg.vsync_pw,
            .vsync_back_porch  = panel_cfg.vsync_bp,
            .vsync_front_porch = panel_cfg.vsync_fp,
        },
        .flags.use_dma2d     = true,
    };

    // 4: vendor config
    ili9881c_vendor_config_t vendor_cfg = {
        .mipi_config = { .dsi_bus = dsi_bus, .dpi_config = &dpi_cfg, .lane_num = panel_cfg.lane_num }
    };

    // 5: panel dev config
    esp_lcd_panel_dev_config_t panel_dev_cfg = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_ele_order  = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = 24,
        .vendor_config  = &vendor_cfg,
    };

    // 6: init panel
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(io_handle, &panel_dev_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    ESP_LOGI(TAG, "Drawing solid blue...");
    // allocate a small tile (e.g. 16x16) and tile it, or fill full
    size_t px_count = panel_cfg.h_size * panel_cfg.v_size;
    size_t buf_size = px_count * 3;
    uint8_t *buf = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    for (size_t i = 0; i < buf_size; i += 3) {
        buf[i+0] = 0x00; // R
        buf[i+1] = 0x00; // G
        buf[i+2] = 0xFF; // B
    }
    // draw the full screen
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, panel_cfg.h_size, panel_cfg.v_size, buf));
    free(buf);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting console display shell...");
    init_console();
    register_console_commands();
    linenoiseClearScreen();

    // Shell loop
    while (true) {
        char *line = linenoise("disp> ");
        if (line == NULL) continue;
        if (strlen(line) > 0) {
            esp_console_run(line, NULL);
            linenoiseHistoryAdd(line);
        }
        free(line);
    }
}

