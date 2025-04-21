idf_component_register(  SRCS           "src/app_twai"
                                        "src/app_motors"
                                        "src/stepper_motors.cpp"
                                        "src/bldc_motors.cpp"
                                        "src/dm3519.cpp"
                         REQUIRES       driver 
                                        freertos
                                        iqmath
                                        FastAccelStepper
                                        lvgl
                         INCLUDE_DIRS   "include" "../../main/ui")