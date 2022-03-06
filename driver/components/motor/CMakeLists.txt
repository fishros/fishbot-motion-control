set(component_srcs "src/motor.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver rotary_encoder  udp_client"
                       REQUIRES "rotary_encoder")
