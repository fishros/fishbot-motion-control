idf_component_register(SRCS "protocol.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES proto_utils uart_protocol udp_client_protocol udp_server_protocol)