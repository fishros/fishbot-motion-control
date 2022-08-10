#include <stdio.h>
#include "esp_system.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "nvs.h"

#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"


static const char *TAG = "NVS_OPERATE";

//数据库的表名
static const char *TB_SELF = "Tb_Self";


void nvs_read_struct(char * struct_name,void * pStruct,uint32_t length){
    nvs_handle mHandleNvsRead;
    esp_err_t err = nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvsRead);
    memset(pStruct, 0x0, length);
    err = nvs_get_blob(mHandleNvsRead, struct_name,pStruct, &length);
    if (err == ESP_OK)
    {
         ESP_LOGI(TAG,"read data success length=%d\n",length);
    }
    //关闭数据库，关闭面板！
    nvs_close(mHandleNvsRead);
}

void nvs_read_string(char * string_name){
    char data[65];
    nvs_handle mHandleNvsRead;
    esp_err_t err = nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvsRead);
    uint32_t len = sizeof(data);
    err = nvs_get_str(mHandleNvsRead, string_name, data, &len);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "get str data = %s ", data);
    else
        ESP_LOGI(TAG, "get str data error");

    //关闭数据库，关闭面板！
    nvs_close(mHandleNvsRead);
}


void nvs_read_uint8(char * i8_name,int8_t *i8){
    nvs_handle mHandleNvsRead;
    esp_err_t err = nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvsRead);    
    //读取 i8
    err = nvs_get_i8(mHandleNvsRead, i8_name, i8);
    if (err == ESP_OK){
        ESP_LOGI(TAG, "get nvs_i8 = %d ", *i8);
    }else{
        ESP_LOGI(TAG, "get nvs_i8 error");
        *i8 = 255;
    }

    //关闭数据库，关闭面板！
    nvs_close(mHandleNvsRead);
}



void nvs_write_uint8(char * i8_name,int8_t i8){
    nvs_handle mHandleNvs;
    esp_err_t err = nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvs);    
   //保存一个 int8_t
    err = nvs_set_i8(mHandleNvs,i8_name,i8);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Save NVS i8 error !!");
    else
        ESP_LOGI(TAG, "Save NVS i8 ok !! nvs_i8 = %d ",i8);

    nvs_commit(mHandleNvs);    
    //关闭数据库，关闭面板！
    nvs_close(mHandleNvs);
}

void nvs_write_string(char * string_name,char * string_value){
    nvs_handle mHandleNvs;
    esp_err_t err = nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvs);
    if (nvs_set_str(mHandleNvs, string_name, string_value) != ESP_OK)
        ESP_LOGE(TAG, "Save NVS String Fail !!  ");
    else
        ESP_LOGI(TAG, "Save NVS String ok !! data : %s ", string_value);

    //提交下！相当于软件面板的 “应用” 按钮，并没关闭面板！
    nvs_commit(mHandleNvs);
    nvs_close(mHandleNvs);
}

void nvs_write_struct(char * struct_name,void * pStruct,uint32_t length){
    nvs_handle mHandleNvs;
    esp_err_t err = nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvs);

    if (nvs_set_blob(mHandleNvs, struct_name,pStruct,length) != ESP_OK)
        ESP_LOGE(TAG, "Save Struct  Fail !!  ");
    else
        ESP_LOGI(TAG, "Save Struct  ok !!  length=%d\n",length);

    //提交下！相当于软件面板的 “应用” 按钮，并没关闭面板！
    nvs_commit(mHandleNvs);
    nvs_close(mHandleNvs);
}
 

 
void nvs_test(void){
    // nvs_write_string("ssid","mweb");
    // nvs_read_string("ssid");
    // device d = {
    //     .id=1,
    //     .hard_id=1234568,
    //     .ssid="mweb",
    //     .pswd="88888888"
    // };
    // nvs_write_struct(&d);
    // nvs_read_struct(&d);
}

