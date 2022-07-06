#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

void i2c_master_init();
void i2c_master_weite_one_byte(uint8_t reg_address, uint8_t data);
void i2c_master_weite_n_bytes(uint8_t reg_address, uint8_t *data,uint8_t data_len);

#endif