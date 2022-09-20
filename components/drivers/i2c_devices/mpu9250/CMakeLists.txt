idf_component_register(SRCS "mpu9250.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES esp32_i2c_rw)