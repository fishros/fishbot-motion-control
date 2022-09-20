/**
 * @brief MPu9250 IMU驱动
 * @author cuizhongren45 (1326986768@qq.com)
 * @version V1.0.0
 * @date 2022-08-22
 * @copyright 版权所有：FishBot Open Source Organization
 * 主要参考：https://github.com/psiphi75/esp-mpu9250
 */
#ifndef __MPU9250_H
#define __MPU9250_H

#include "freertos/FreeRTOS.h"
#include "esp32_i2c_rw.h"
#include <math.h>

#define MPU9250_I2C_ADDRESS_AD0_LOW (0x68)
#define MPU9250_I2C_ADDR MPU9250_I2C_ADDRESS_AD0_LOW
#define MPU9250_I2C_ADDRESS_AD0_HIGH (0x69)
#define MPU9250_WHO_AM_I (0x75)

#define MPU9250_RA_CONFIG (0x1A)
#define MPU9250_RA_GYRO_CONFIG (0x1B)
#define MPU9250_RA_ACCEL_CONFIG_1 (0x1C)
#define MPU9250_RA_ACCEL_CONFIG_2 (0x1D)

#define MPU9250_RA_INT_PIN_CFG (0x37)

#define MPU9250_INTCFG_ACTL_BIT (7)
#define MPU9250_INTCFG_OPEN_BIT (6)
#define MPU9250_INTCFG_LATCH_INT_EN_BIT (5)
#define MPU9250_INTCFG_INT_ANYRD_2CLEAR_BIT (4)
#define MPU9250_INTCFG_ACTL_FSYNC_BIT (3)
#define MPU9250_INTCFG_FSYNC_INT_MODE_EN_BIT (2)
#define MPU9250_INTCFG_BYPASS_EN_BIT (1)
#define MPU9250_INTCFG_NONE_BIT (0)

#define MPU9250_ACCEL_XOUT_H (0x3B)
#define MPU9250_ACCEL_XOUT_L (0x3C)
#define MPU9250_ACCEL_YOUT_H (0x3D)
#define MPU9250_ACCEL_YOUT_L (0x3E)
#define MPU9250_ACCEL_ZOUT_H (0x3F)
#define MPU9250_ACCEL_ZOUT_L (0x40)
#define MPU9250_TEMP_OUT_H (0x41)
#define MPU9250_TEMP_OUT_L (0x42)
#define MPU9250_GYRO_XOUT_H (0x43)
#define MPU9250_GYRO_XOUT_L (0x44)
#define MPU9250_GYRO_YOUT_H (0x45)
#define MPU9250_GYRO_YOUT_L (0x46)
#define MPU9250_GYRO_ZOUT_H (0x47)
#define MPU9250_GYRO_ZOUT_L (0x48)

#define MPU9250_RA_USER_CTRL (0x6A)
#define MPU9250_RA_PWR_MGMT_1 (0x6B)
#define MPU9250_RA_PWR_MGMT_2 (0x6C)
#define MPU9250_PWR1_DEVICE_RESET_BIT (7)
#define MPU9250_PWR1_SLEEP_BIT (6)
#define MPU9250_PWR1_CYCLE_BIT (5)
#define MPU9250_PWR1_TEMP_DIS_BIT (3)
#define MPU9250_PWR1_CLKSEL_BIT (0)
#define MPU9250_PWR1_CLKSEL_LENGTH (3)

#define MPU9250_GCONFIG_FS_SEL_BIT (3)
#define MPU9250_GCONFIG_FS_SEL_LENGTH (2)
#define MPU9250_GYRO_FS_250 (0x00)
#define MPU9250_GYRO_FS_500 (0x01)
#define MPU9250_GYRO_FS_1000 (0x02)
#define MPU9250_GYRO_FS_2000 (0x03)
#define MPU9250_GYRO_SCALE_FACTOR_0 (131)
#define MPU9250_GYRO_SCALE_FACTOR_1 (65.5)
#define MPU9250_GYRO_SCALE_FACTOR_2 (32.8)
#define MPU9250_GYRO_SCALE_FACTOR_3 (16.4)

#define MPU9250_ACONFIG_FS_SEL_BIT (3)
#define MPU9250_ACONFIG_FS_SEL_LENGTH (2)
#define MPU9250_ACCEL_FS_2 (0x00)
#define MPU9250_ACCEL_FS_4 (0x01)
#define MPU9250_ACCEL_FS_8 (0x02)
#define MPU9250_ACCEL_FS_16 (0x03)
#define MPU9250_ACCEL_SCALE_FACTOR_0 (16384)
#define MPU9250_ACCEL_SCALE_FACTOR_1 (8192)
#define MPU9250_ACCEL_SCALE_FACTOR_2 (4096)
#define MPU9250_ACCEL_SCALE_FACTOR_3 (2048)

#define MPU9250_CLOCK_INTERNAL (0x00)
#define MPU9250_CLOCK_PLL_XGYRO (0x01)
#define MPU9250_CLOCK_PLL_YGYRO (0x02)
#define MPU9250_CLOCK_PLL_ZGYRO (0x03)
#define MPU9250_CLOCK_KEEP_RESET (0x07)
#define MPU9250_CLOCK_PLL_EXT32K (0x04)
#define MPU9250_CLOCK_PLL_EXT19M (0x05)

#define MPU9250_I2C_SLV0_DO (0x63)
#define MPU9250_I2C_SLV1_DO (0x64)
#define MPU9250_I2C_SLV2_DO (0x65)

#define MPU9250_USERCTRL_DMP_EN_BIT (7)
#define MPU9250_USERCTRL_FIFO_EN_BIT (6)
#define MPU9250_USERCTRL_I2C_MST_EN_BIT (5)
#define MPU9250_USERCTRL_I2C_IF_DIS_BIT (4)
#define MPU9250_USERCTRL_DMP_RESET_BIT (3)
#define MPU9250_USERCTRL_FIFO_RESET_BIT (2)
#define MPU9250_USERCTRL_I2C_MST_RESET_BIT (1)
#define MPU9250_USERCTRL_SIG_COND_RESET_BIT (0)


#define AK8963_ADDRESS (0x0c)
#define AK8963_WHO_AM_I (0x00) // should return 0x48
#define AK8963_WHO_AM_I_RESPONSE (0x48)
#define AK8963_INFO (0x01)
#define AK8963_ST1 (0x02)    // data ready status bit 0
#define AK8963_XOUT_L (0x03) // data
#define AK8963_XOUT_H (0x04)
#define AK8963_YOUT_L (0x05)
#define AK8963_YOUT_H (0x06)
#define AK8963_ZOUT_L (0x07)
#define AK8963_ZOUT_H (0x08)
#define AK8963_ST2 (0x09)    // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL (0x0a)   // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC (0x0c)   // Self test control
#define AK8963_I2CDIS (0x0f) // I2C disable
#define AK8963_ASAX (0x10)   // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY (0x11)   // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ (0x12)

#define AK8963_ST1_DRDY_BIT (0)
#define AK8963_ST1_DOR_BIT (1)

#define AK8963_CNTL_MODE_OFF (0x00)                // Power-down mode
#define AK8963_CNTL_MODE_SINGLE_MEASURE (0x01)     // Single measurement mode
#define AK8963_CNTL_MODE_CONTINUE_MEASURE_1 (0x02) // Continuous measurement mode 1 - Sensor is measured periodically at 8Hz
#define AK8963_CNTL_MODE_CONTINUE_MEASURE_2 (0x06) // Continuous measurement mode 2 - Sensor is measured periodically at 100Hz
#define AK8963_CNTL_MODE_EXT_TRIG_MEASURE (0x04)   // External trigger measurement mode
#define AK8963_CNTL_MODE_SELF_TEST_MODE (0x08)     // Self-test mode
#define AK8963_CNTL_MODE_FUSE_ROM_ACCESS (0x0f)    // Fuse ROM access mode

#define BYTE_2_INT_BE(byte, i) ((int16_t)((byte[i] << 8) + (byte[i + 1])))
#define BYTE_2_INT_LE(byte, i) ((int16_t)((byte[i + 1] << 8) + (byte[i])))

#define DEG2RAD(deg) (deg * M_PI / 180.0f)

typedef struct
{
  float x, y, z;
} vector_t;

typedef struct
{
  // Magnetometer
  vector_t mag_offset;
  vector_t mag_scale;

  // Gryoscope
  vector_t gyro_bias_offset;

  // Accelerometer
  vector_t accel_offset;
  vector_t accel_scale_lo;
  vector_t accel_scale_hi;

} calibration_t;

uint8_t mpu9250_device_address;
uint8_t buffer[14];

/**
 * @brief MPU9250初始化函数
 * 
 */
void mpu9250_init();

/**
 * @brief 设置mpu9250时钟
 * 
 * @param source 
 */
void mpu9250_set_clock_source(uint8_t source);

/**
 * @brief 设置gyro的参数
 * 
 * @param range 
 */
void mpu9250_set_full_scale_gyro_range(uint8_t range);

/**
 * @brief 设置accel的参数
 * 
 * @param range 
 */
void mpu9250_set_full_scale_accel_range(uint8_t range);

/**
 * @brief 设置mpu9250睡眠模式
 * 
 * @param enabled 
 */
void mpu9250_set_sleep_enabled(bool enabled);

/**
 * @brief 设置mpu9250 iic主从模式
 * 
 * @param enabled 
 */
void mpu9250_set_i2c_master_mode_enabled(bool enabled);

/**
 * @brief 设置iic的bypass使能
 * 
 * @param enabled 
 */
void mpu9250_set_i2c_bypass_enabled(bool enabled);

/**
 * @brief 查看bypass的状态
 * 
 * @return true 
 * @return false 
 */
bool mpu9250_get_i2c_bypass_enabled();

/**
 * @brief 获取ak8963磁力计采样模式
 * 
 * @param mode 
 * @return true 
 * @return false 
 */
bool ak8963_get_cntl(uint8_t *mode);

/**
 * @brief 设置ak8963磁力计采样模式
 * 
 * @param mode 
 * @return true 
 * @return false 
 */
bool ak8963_set_cntl(uint8_t mode);

/**
 * @brief 磁力计数据滤波（不需要了）
 * 
 */
void ak8963_get_sensitivity_adjustment_values();

/**
 * @brief ak8963磁力计初始化
 * 
 * @return true 
 * @return false 
 */
bool ak8963_init();

/**
 * @brief 使能磁力计
 * 
 * @return true 
 * @return false 
 */
bool enable_magnetometer();

/**
 * @brief mpu9250任务初始化
 * 
 * @return true 
 * @return false 
 */
bool mpu9250_task_init();

/**
 * @brief 获取mpu9250的温度-摄氏度
 * 
 * @param val 温度数据-摄氏度
 */
void get_temperature_celsius(float *val);

/**
 * @brief 获取mpu9250温度的原始数据
 * 
 * @param val 原始数据
 */
void get_temperature_raw(uint16_t *val);

/**
 * @brief 获取ak8963磁力计的数据
 * 
 * @param v xyz三轴方向上磁力的大小
 * @return true 
 * @return false 
 */
bool ak8963_get_mag(vector_t *v);

/**
 * @brief xyz三轴方向上磁力的原始数据
 * 
 * @param bytes 
 * @return true 
 * @return false 
 */
bool ak8963_get_mag_raw(uint8_t bytes[6]);

/**
 * @brief 获取accl加速度计、gyro陀螺仪数据
 * 
 * @param va 三轴加速度数据
 * @param vg 三轴陀螺仪数据
 */
void get_accel_gyro(vector_t *va, vector_t *vg);

/**
 * @brief 将三轴加速度计数据存到v中
 * 
 * @param bytes 
 * @param v 
 */
void align_accel(uint8_t bytes[6], vector_t *v);

/**
 * @brief 将三轴陀螺仪数据存到v中
 * 
 * @param bytes 
 * @param v 
 */
void align_gryo(uint8_t bytes[6], vector_t *v);

/**
 * @brief 获取accl、gyro、mag数据
 * 
 * @param va 三轴accl
 * @param vg 三轴gyro
 * @param vm 三轴mag
 */
void get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm);

/**
 * @brief 快速求x的开根号
 * 
 * @param x 
 * @return float 
 */
float invSqrt(float x);

/**
 * @brief 计算accl和gyro的零偏
 * 
 */
void cal_offset();

/**
 * @brief mpu9250的任务函数
 * 
 */
void mpu9250_task(void);

/**
 * @brief 获取mpu9250经过AHRS算法处理后的欧拉角数据，yaw有磁力计纠正
 * 
 * @param param 
 */
void get_mpu9250_euler_angle(void *param);

/**
 * @brief 计算磁力计数据零偏
 * 
 */
void ak8963_caloffset();

/**
 * @brief 求最大值
 * 
 * @param x 
 * @param y 
 * @return float 
 */
float _MAX(float x, float y);

/**
 * @brief 求最小值
 * 
 * @param x 
 * @param y 
 * @return float 
 */
float _MIN(float x, float y);

/**
 * @brief 含有磁力计纠正的AHRS算法
 * 
 * @param gx gyro.x
 * @param gy gyro.y
 * @param gz gyro.z
 * @param ax acc.x
 * @param ay acc.y
 * @param az acc.z
 * @param mx mag.x
 * @param my mag.y
 * @param mz mag.z
 */
void IMU_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#endif 
