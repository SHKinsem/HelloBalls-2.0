set(requires    fatfs
                spiffs)

set(srcs    "twai_network_example_master_main.c"
            "esp32_s3_szp.c" 
            # "dji_rm_motors.cpp" 
            "app_ui.c"
            "app_button.c")

idf_component_register(SRCS ${srcs}
                       REQUIRES ${requires}
                       WHOLE_ARCHIVE
                       INCLUDE_DIRS ".")
