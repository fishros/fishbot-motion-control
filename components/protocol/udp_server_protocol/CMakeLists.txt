set(component_srcs "udp_server_protocol.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       REQUIRES proto_utils)
