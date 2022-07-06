idf_component_register(SRCS "motor.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES rotary_encoder pid_controller)