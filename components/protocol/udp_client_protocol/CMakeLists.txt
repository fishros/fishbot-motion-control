set(component_srcs "udp_client.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       REQUIRES "nvs_flash")
