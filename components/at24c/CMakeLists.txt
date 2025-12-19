# components/at24c/CMakeLists.txt
idf_component_register(
    SRCS "at24c.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver i2cdev
)