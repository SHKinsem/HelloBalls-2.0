idf_component_register( SRCS            "debugger.cpp"
                        REQUIRES        driver 
                                        motors
                                        szp_bsp
                                        buttons
                        INCLUDE_DIRS    "." "../../main/ui")