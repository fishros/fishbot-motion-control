
#include "stdio.h"
typedef struct
{
    char data_id;        // 数据编号ID
    short int  data_len;      // 数据长度 2
    char data_direction; // 数据方向
}  __attribute__((packed)) proto_data_header_t;


int main()
{
    short int a=1;
    printf("%d\n",sizeof(a));
    printf("%d\n",sizeof(proto_data_header_t));
    return 0;
}