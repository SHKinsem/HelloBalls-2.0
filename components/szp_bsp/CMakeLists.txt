idf_component_register( SRCS            "src/esp32_s3_szp.c"
                        REQUIRES        driver
                                        fatfs 
                                        spiffs
                                        esp_lvgl_port
                                        esp_lcd_touch_ft5x06
                                        esp_codec_dev
                        INCLUDE_DIRS    "include")