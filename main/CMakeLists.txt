#Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(SRCS "main.cpp" "state_machine.cpp" ${SRC_UI}
                       WHOLE_ARCHIVE
                       INCLUDE_DIRS "." "ui")
                       
