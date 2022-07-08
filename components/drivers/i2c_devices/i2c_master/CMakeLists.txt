idf_component_register(SRCS "i2c_master.c" "oled.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES rotary_encoder)