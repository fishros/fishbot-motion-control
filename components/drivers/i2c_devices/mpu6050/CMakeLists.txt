idf_component_register(SRCS "mpu6050.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES esp32_i2c_rw)