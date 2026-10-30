# The codec has no dependencies; the NimBLE transport is built only when NimBLE is enabled.
set(srcs "src/ble_remote_codec.c")
if(CONFIG_BT_NIMBLE_ENABLED)
    list(APPEND srcs "src/ble_remote.c")
endif()

idf_component_register( SRCS            ${srcs}
                        INCLUDE_DIRS    "include"
                        REQUIRES        bt
                        PRIV_REQUIRES   nvs_flash
                                        esp_hw_support
                                        freertos
                                        log)
