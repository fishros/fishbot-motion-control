#ifndef __NVS_H__
#define __NVS_H__

void nvs_test(void);

void nvs_read_string(char * string_name);
void nvs_read_struct(char * struct_name,void * pStruct,uint32_t length);

void nvs_write_string(char * string_name,char * string_value);
void nvs_write_struct(char * struct_name,void * pStruct,uint32_t length);

/*写 */
void nvs_write_uint8(char * i8_name,int8_t i8);

/*读取int */
void nvs_read_uint8(char * i8_name,int8_t *i8);


#endif