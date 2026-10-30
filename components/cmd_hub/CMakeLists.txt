idf_component_register( SRCS            "src/cmd_hub.c"
                        INCLUDE_DIRS    "include"
                        REQUIRES        freertos
                        PRIV_REQUIRES   esp_timer
                                        log)
