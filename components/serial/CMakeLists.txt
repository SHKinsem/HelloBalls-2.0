idf_component_register( SRCS            "src/serial.c"
                        REQUIRES        driver 
                                        esp_system
                                        freertos
                                        esp_driver_usb_serial_jtag
                                        esp_timer
                                        leds
                                        szp_bsp
                        INCLUDE_DIRS    "include")