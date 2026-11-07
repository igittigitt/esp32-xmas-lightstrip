# components/at24c/CMakeLists.txt
idf_component_register(
    SRCS "led_controller.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)