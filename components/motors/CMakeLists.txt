idf_component_register(  SRCS           "src/app_twai"
                                        "src/app_motors"
                                        "src/stepper_motors.cpp"
                                        "src/bldc_motors.cpp"
                                        "src/dm3519.cpp"
                                        "src/can_channel.cpp"
                         REQUIRES       driver 
                                        freertos
                                        iqmath
                                        FastAccelStepper
                                        lvgl
                                        serial
                                        szp_bsp
                         INCLUDE_DIRS   "include" "../../main/ui")