idf_component_register(SRCS "fishbot.c" "fishbot_config.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES led motor oled esp32_i2c_rw)