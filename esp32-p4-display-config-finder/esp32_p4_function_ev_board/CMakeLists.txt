
idf_component_register(
    SRCS "esp32_p4_function_ev_board.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver vfs fatfs
    PRIV_REQUIRES esp_lcd usb spiffs espressif__esp_lcd_ili9881c
)
