# components/at24c/CMakeLists.txt
idf_component_register(
    SRCS "at24c.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver i2cdev
)