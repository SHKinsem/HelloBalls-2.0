idf_component_register(  SRCS           "src/app_ui.c"
                                        "src/unomiku_320x198.c"
                         REQUIRES       esp-audio-player
                                        driver
                                        lvgl
                                        szp_bsp
                                        esp-file-iterator
                         INCLUDE_DIRS   "include")