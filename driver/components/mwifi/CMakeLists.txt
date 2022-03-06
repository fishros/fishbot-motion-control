set(component_srcs "src/mwifi.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver rotary_encoder"
                       REQUIRES "nvs_flash")
