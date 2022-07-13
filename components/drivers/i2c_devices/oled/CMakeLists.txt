idf_component_register(SRCS "oled.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES esp32_i2c_rw)