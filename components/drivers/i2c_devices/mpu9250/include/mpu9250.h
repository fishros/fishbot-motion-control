
#ifndef __MPU9250_H
#define __MPU9250_H

#include "freertos/FreeRTOS.h"
#include "esp32_i2c_rw.h"
#include <math.h>

/*****************/
/** MPU9250 MAP **/
/*****************/
// documentation:
//   https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf
//   https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf

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

// esp_err_t i2c_mpu9250_init(calibration_t *cal);
// esp_err_t set_clock_source(uint8_t adrs);
// esp_err_t set_full_scale_gyro_range(uint8_t adrs);
// esp_err_t set_full_scale_accel_range(uint8_t adrs);
// esp_err_t set_sleep_enabled(bool state);
// esp_err_t get_device_id(uint8_t *val);
// esp_err_t get_temperature_raw(uint16_t *val);
// esp_err_t get_temperature_celsius(float *val);

// esp_err_t get_bypass_enabled(bool *state);
// esp_err_t set_bypass_enabled(bool state);
// esp_err_t get_i2c_master_mode(bool *state);
// esp_err_t set_i2c_master_mode(bool state);

// esp_err_t get_accel(vector_t *v);
// esp_err_t get_gyro(vector_t *v);
// esp_err_t get_mag(vector_t *v);
// esp_err_t get_accel_gyro(vector_t *va, vector_t *vg);
// esp_err_t get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm);
// esp_err_t get_mag_raw(uint8_t bytes[6]);

void print_settings(void);
uint8_t buffer[14];
//新功能函数

void mpu9250_init();
void mpu9250_set_clock_source(uint8_t source);
void mpu9250_set_full_scale_gyro_range(uint8_t range);
void mpu9250_set_full_scale_accel_range(uint8_t range);
void mpu9250_set_sleep_enabled(bool enabled);
void mpu9250_set_i2c_master_mode_enabled(bool enabled);
void mpu9250_set_i2c_bypass_enabled(bool enabled);
bool mpu9250_get_i2c_bypass_enabled();
bool ak8963_get_cntl(uint8_t *mode);
bool ak8963_set_cntl(uint8_t mode);
void ak8963_get_sensitivity_adjustment_values();
bool ak8963_init();
bool enable_magnetometer();
// int8_t ak8963_get_device_id(uint8_t *val);
void ak8963_print_settings(void);
bool mpu9250_task_init();
void MadgwickAHRSinit(float sampleFreqDef, float betaDef);
void mpu9250_run(void );
void get_temperature_celsius(float *val);
void get_temperature_raw(uint16_t *val);
bool ak8963_get_mag(vector_t *v);
bool ak8963_get_mag_raw(uint8_t bytes[6]);
void get_accel_gyro(vector_t *va, vector_t *vg);
void align_accel(uint8_t bytes[6], vector_t *v);
void align_gryo(uint8_t bytes[6], vector_t *v);
float scale_accel(float value, float offset, float scale_lo, float scale_hi);
void get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm);
static void transform_accel_gyro(vector_t *v);
static void transform_mag(vector_t *v);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);
void MadgwickGetVector(float *angle, float *x, float *y, float *z);
float norm_angle_0_2pi(float a);
void MadgwickGetEulerAngles(float *heading, float *pitch, float *roll);
void MadgwickGetEulerAnglesDegrees(float *heading, float *pitch, float *roll);


#endif // __MPU9250_H
