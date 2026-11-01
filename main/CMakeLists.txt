#Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(SRCS "main.cpp" "state_machine.cpp" "remote_link.cpp" "robot_params.cpp" "od_table.cpp" "od_server.cpp" "motor_debug.cpp" "health.cpp" "coredump_wdt.c" ${SRC_UI}
                       WHOLE_ARCHIVE
                       INCLUDE_DIRS "." "ui")
                       

# Clear a pending watchdog interrupt before the core dump is written (coredump_wdt.c)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_core_dump_write")
