set(component_srcs "proto_utils.c" "proto_v1.0.0.20220621.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       REQUIRES )
