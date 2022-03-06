set(component_srcs "src/udp_client.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "nvs_flash coap asio"
                       REQUIRES "motor")

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
set(EXCLUDE_COMPONENTS "coap" "asio")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)