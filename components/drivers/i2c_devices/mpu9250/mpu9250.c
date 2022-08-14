#include "mpu9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21     /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

static const char *TAG = "mpu9250";

static bool initialised = false;

static float gyro_inv_scale = 1.0;
static float accel_inv_scale = 1.0;


volatile float sampleFreq = 50;                            // 2 * proportional gain (Kp)
volatile float beta = 0.8;                                 // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},

    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}};


vector_t asa;
uint64_t i = 0;
//static esp_err_t enable_magnetometer(void);

void mpu9250_init()
{

    uint8_t mpu9250_id;
    mpu9250_device_address = MPU9250_I2C_ADDR;

    for(int i = 0;i<10;i++)
    {
    esp32_i2c_read_byte(MPU9250_I2C_ADDR,MPU9250_WHO_AM_I,&mpu9250_id);
    ESP_LOGE(TAG, "mpu9250---id:::::::%d",mpu9250_id);  
    }
    //cal = c;
    esp32_i2c_write_bit(MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, 1);
    vTaskDelay(10 / portTICK_RATE_MS);

    //mpu9250_set_clock_source(MPU9250_CLOCK_PLL_XGYRO);
    esp32_i2c_write_bits(MPU9250_I2C_ADDR, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, MPU9250_CLOCK_PLL_XGYRO);
    vTaskDelay(10 / portTICK_RATE_MS);

    mpu9250_set_full_scale_gyro_range(MPU9250_GYRO_FS_250);
    vTaskDelay(10 / portTICK_RATE_MS);

    mpu9250_set_full_scale_accel_range(MPU9250_ACCEL_FS_4);
    vTaskDelay(10 / portTICK_RATE_MS);

    mpu9250_set_sleep_enabled(0);
    vTaskDelay(10 / portTICK_RATE_MS);
    ESP_LOGE(TAG, "END of MPU9250 initialization");
    

    for(int i = 0;i<10;i++)
    {
    esp32_i2c_read_byte(MPU9250_I2C_ADDR,MPU9250_WHO_AM_I,&mpu9250_id);
    ESP_LOGE(TAG, "mpu9250---2222222id:%d",mpu9250_id);  
    }


    bool mag_ret = enable_magnetometer();
    if(mag_ret == true)
    {
      ESP_LOGE(TAG, "enable_magnetometer success"); 
    }
    else
    {
      ESP_LOGE(TAG, "enable_magnetometer failed"); 
    }

}


void mpu9250_set_clock_source(uint8_t source)
{
    esp32_i2c_write_bits(
        mpu9250_device_address,
        MPU9250_RA_PWR_MGMT_1,
        MPU9250_PWR1_DEVICE_RESET_BIT,
        1,
        source);
}

void mpu9250_set_full_scale_gyro_range(uint8_t range)
{
    esp32_i2c_write_bits(
        mpu9250_device_address,
        MPU9250_RA_GYRO_CONFIG,
        MPU9250_GCONFIG_FS_SEL_BIT,
        MPU9250_GCONFIG_FS_SEL_LENGTH,
        range);
}


void mpu9250_set_full_scale_accel_range(uint8_t range)
{
    esp32_i2c_write_bits(
        mpu9250_device_address,
        MPU9250_RA_ACCEL_CONFIG_1,
        MPU9250_ACONFIG_FS_SEL_BIT, MPU9250_ACONFIG_FS_SEL_LENGTH,
        range);
}


void mpu9250_set_sleep_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu9250_device_address,
        MPU9250_RA_PWR_MGMT_1,
        MPU9250_PWR1_SLEEP_BIT,
        enabled);
}

void mpu9250_set_i2c_master_mode_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu9250_device_address,
        MPU9250_RA_USER_CTRL,
        MPU9250_USERCTRL_I2C_MST_EN_BIT,
        enabled);
}


void mpu9250_set_i2c_bypass_enabled(bool enabled)
{
    esp32_i2c_write_bit(
        mpu9250_device_address,
        MPU9250_RA_INT_PIN_CFG,
        MPU9250_INTCFG_BYPASS_EN_BIT,
        enabled);
}

bool mpu9250_get_i2c_bypass_enabled()
{
    esp32_i2c_read_bit(
        mpu9250_device_address,
        MPU9250_RA_INT_PIN_CFG,
        MPU9250_INTCFG_BYPASS_EN_BIT,
        buffer);

    return (buffer[0]);
}

bool ak8963_get_cntl(uint8_t *mode)
{
  return esp32_i2c_read_byte( AK8963_ADDRESS, AK8963_CNTL, mode);
}

bool ak8963_set_cntl(uint8_t mode)
{
  return esp32_i2c_write_byte(AK8963_ADDRESS, AK8963_CNTL, mode);
}

void ak8963_get_sensitivity_adjustment_values()
{
  // Need to set to Fuse mode to get valid values from this.
  uint8_t current_mode;
  ak8963_get_cntl(&current_mode);

  ak8963_set_cntl(AK8963_CNTL_MODE_FUSE_ROM_ACCESS);

  vTaskDelay(20 / portTICK_RATE_MS);

  uint8_t xi, yi, zi;
  esp32_i2c_read_byte( AK8963_ADDRESS, AK8963_ASAX, &xi);

  esp32_i2c_read_byte(AK8963_ADDRESS, AK8963_ASAY, &yi);

  esp32_i2c_read_byte(AK8963_ADDRESS, AK8963_ASAZ, &zi);


  // Get the ASA* values
  asa.x = (((float)xi - 128.0) * 0.5) / 128.0 + 1.0;
  asa.y = (((float)yi - 128.0) * 0.5) / 128.0 + 1.0;
  asa.z = (((float)zi - 128.0) * 0.5) / 128.0 + 1.0;

  ak8963_set_cntl(current_mode);
}

//气压计初始化
bool ak8963_init()
{
  // if(initialised)
  // {
  //   ESP_LOGE(TAG, "ak8963_init has already been called"); 
  //   return false;
  // }

  uint8_t id;
  while (id != 0x02) 
  {
    esp32_i2c_read_byte(AK8963_ADDRESS,AK8963_WHO_AM_I,&id);
    ESP_LOGE(TAG, "id:%02d",id);
    if(id == AK8963_WHO_AM_I_RESPONSE)
    {
      ESP_LOGE(TAG, "id==0x48 okk");
      break;
    }
  }


  if(id & 0x02)  //AK8963_WHO_AM_I_RESPONSE
  {
    ak8963_get_sensitivity_adjustment_values();
    vTaskDelay(10 / portTICK_RATE_MS);
    ak8963_set_cntl(AK8963_CNTL_MODE_CONTINUE_MEASURE_2);
    initialised = true;
    return true;
  }
  else
  {
    ESP_LOGE(TAG, "AK8963: Device ID is not equal to 0x%02x, device value is 0x%02x", AK8963_WHO_AM_I_RESPONSE, id);
    return false;
  }
}

// int8_t ak8963_get_device_id(uint8_t *val)
// {
//   return esp32_i2c_read_byte(AK8963_ADDRESS, AK8963_WHO_AM_I, val);
// }

//使能气压计
bool enable_magnetometer()
{
  ESP_LOGI(TAG, "Enabling magnetometer");

  mpu9250_set_i2c_master_mode_enabled(false);
  vTaskDelay(100 / portTICK_RATE_MS);

  mpu9250_set_i2c_bypass_enabled(true);
  vTaskDelay(100 / portTICK_RATE_MS);

  bool is_enabled = mpu9250_get_i2c_bypass_enabled();
  if (is_enabled)
  {
    //ak8963大气针初始化
    ak8963_init();
    ak8963_print_settings();
    ESP_LOGI(TAG, "Magnetometer enabled");
    return true;
  }
  else
  {
    ESP_LOGE(TAG, "Can't turn on RA_INT_PIN_CFG.");
    return false;
  }

}


void ak8963_print_settings(void)
{
  // char *cntl_modes[] = {"0x00 (Power-down mode)",
  //                       "0x01 (Single measurement mode)",
  //                       "0x02 (Continuous measurement mode 1: 8Hz)",
  //                       "0x03 Invalid mode",
  //                       "0x04 (External trigger measurement mode)",
  //                       "0x05 Invalid mode",
  //                       "0x06 (Continuous measurement mode 2: 100Hz)",
  //                       "0x07 Invalid mode",
  //                       "0x08 (Self-test mode)",
  //                       "0x09 Invalid mode",
  //                       "0x0A Invalid mode",
  //                       "0x0B Invalid mode",
  //                       "0x0C Invalid mode",
  //                       "0x0D Invalid mode",
  //                       "0x0E Invalid mode",
  //                       "0x0F Invalid mode",
  //                       "0x0F (Fuse ROM access mode)"};

  // uint8_t device_id;
  // esp32_i2c_read_byte(AK8963_ADDRESS,AK8963_WHO_AM_I,&device_id);

  // uint8_t cntl;
  // ak8963_get_cntl(&cntl);

  // ESP_LOGI(TAG, "Magnetometer (Compass):");

  // ESP_LOGI(TAG, "--> initialised: %s", initialised ? "true" : "false");
  // ESP_LOGI(TAG, "--> Device ID: 0x%02x", device_id);
  // ESP_LOGI(TAG, "--> Mode: %s", cntl_modes[cntl]);
  // ESP_LOGI(TAG, "--> ASA Scalars:");
  // ESP_LOGI(TAG, "  --> x: %f", asa.x);
  // ESP_LOGI(TAG, "  --> y: %f", asa.y);
  // ESP_LOGI(TAG, "  --> z: %f", asa.z);
  // ESP_LOGI(TAG, "--> Offset:");
  // ESP_LOGI(TAG, "  --> x: %f", cal->mag_offset.x);
  // ESP_LOGI(TAG, "  --> y: %f", cal->mag_offset.y);
  // ESP_LOGI(TAG, "  --> z: %f", cal->mag_offset.z);
  // ESP_LOGI(TAG, "--> Scale:");
  // ESP_LOGI(TAG, "  --> x: %f", cal->mag_scale.x);
  // ESP_LOGI(TAG, "  --> y: %f", cal->mag_scale.y);
  // ESP_LOGI(TAG, "  --> z: %f", cal->mag_scale.z);
}



bool mpu9250_task_init(void)
{
    mpu9250_init(&cal);
    MadgwickAHRSinit(50, 0.8);

    mpu9250_run();
    return true;
}

void MadgwickAHRSinit(float sampleFreqDef, float betaDef)
{
  sampleFreq = sampleFreqDef;
  beta = betaDef;
}

void mpu9250_run(void )
{
  
  while(true)
  { 
    vector_t va, vg, vm;
    //获取原始数据
    get_accel_gyro_mag(&va, &vg, &vm);
    //转化数据
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);
    //应用AHRS算法
    MadgwickAHRSupdate(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                       va.x, va.y, va.z,
                       vm.x, vm.y, vm.z);
    float heading, pitch, roll;
    MadgwickGetEulerAnglesDegrees(&heading, &pitch, &roll);
    ESP_LOGI(TAG, "heading: %2.3f, pitch: %2.3f, roll: %2.3f", heading, pitch, roll);

    // 获得温度，可以检验mpu9250的读写是否正常
    // float temp;
    // get_temperature_celsius(&temp);   
    // ESP_LOGI(TAG, "Temp %2.3f",temp);


  
  }
  

}

void get_temperature_celsius(float *val)
{
  uint16_t raw_temp;
  get_temperature_raw(&raw_temp);

  *val = ((float)raw_temp) / 333.87 + 21.0;
}

void get_temperature_raw(uint16_t *val)
{
  uint8_t bytes[2];
  esp32_i2c_read_bytes(MPU9250_I2C_ADDR, MPU9250_TEMP_OUT_H, 2, bytes);

  *val = BYTE_2_INT_BE(bytes, 0);
  
}


bool ak8963_get_mag(vector_t *v)
{
  bool ret;
  uint8_t bytes[6];
  ret = ak8963_get_mag_raw(bytes);
  if(ret == true)
  {
    float xi = (float)BYTE_2_INT_LE(bytes, 0);
    float yi = (float)BYTE_2_INT_LE(bytes, 2);
    float zi = (float)BYTE_2_INT_LE(bytes, 4);

    v->x = (xi * asa.x - cal.mag_offset.x) * cal.mag_scale.x;
    v->y = (yi * asa.y - cal.mag_offset.y) * cal.mag_scale.y;
    v->z = (zi * asa.z - cal.mag_offset.z) * cal.mag_scale.z;
    return true;
  }
  else
  {
    return false;
  }
}

bool ak8963_get_mag_raw(uint8_t bytes[6])
{
  esp32_i2c_read_bytes(AK8963_ADDRESS, AK8963_XOUT_L, 6, bytes);
  vTaskDelay(1 / portTICK_RATE_MS);
  uint8_t b;
  esp32_i2c_read_byte(AK8963_ADDRESS, AK8963_ST2, &b);
  return true;
}

void get_accel_gyro(vector_t *va, vector_t *vg)
{
  uint8_t bytes[14];
  esp32_i2c_read_bytes(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, 14, bytes);

  align_accel(bytes, va);
  align_gryo(&bytes[8], vg);

}

void align_accel(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);

  v->x = scale_accel((float)xi, cal.accel_offset.x, cal.accel_scale_lo.x, cal.accel_scale_hi.x);
  v->y = scale_accel((float)yi, cal.accel_offset.y, cal.accel_scale_lo.y, cal.accel_scale_hi.y);
  v->z = scale_accel((float)zi, cal.accel_offset.z, cal.accel_scale_lo.z, cal.accel_scale_hi.z);
}

void align_gryo(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);

  v->x = (float)xi * gyro_inv_scale + cal.gyro_bias_offset.x;
  v->y = (float)yi * gyro_inv_scale + cal.gyro_bias_offset.y;
  v->z = (float)zi * gyro_inv_scale + cal.gyro_bias_offset.z;
}

float scale_accel(float value, float offset, float scale_lo, float scale_hi)
{
  if (value < 0)
  {
    return -(value * accel_inv_scale - offset) / (scale_lo - offset);
  }
  else
  {
    return (value * accel_inv_scale - offset) / (scale_hi - offset);
  }
}

void get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm)
{
  get_accel_gyro(va, vg);
  ak8963_get_mag(vm);
}

static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}


void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
  {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/**
 * Convert the quaternion to a vector with angle.  Reverse of the code
 * in the following link: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
 * @return {object} Normalised vector - {x, y, z, angle}
 */
void MadgwickGetVector(float *angle, float *x, float *y, float *z)
{
  float ang = 2.0 * acos(q0);
  float sin_angle = sin(ang / 2.0);
  *angle = ang;
  *x = q1 / sin_angle;
  *y = q2 / sin_angle;
  *z = q3 / sin_angle;
};

float norm_angle_0_2pi(float a)
{
  a = fmod(a, M_PI * 2.0);
  if (a < 0)
  {
    a += M_PI * 2.0;
  }
  return a;
}

/**
 * Return an object with the Euler angles {heading; pitch, roll}, in radians.
 *
 * Where:
 *   - heading is from magnetic north, going west (about z-axis).
 *   - pitch is from vertical, going forward (about y-axis).
 *   - roll is from vertical, going right (about x-axis).
 *
 * Thanks to:
 *   https://github.com/PenguPilot/PenguPilot/blob/master/autopilot/service/util/math/quat.c#L103
 * @return {object} {heading, pitch, roll} in radians
 */
void MadgwickGetEulerAngles(float *heading, float *pitch, float *roll)
{
  float ww = q0 * q0;
  float xx = q1 * q1;
  float yy = q2 * q2;
  float zz = q3 * q3;
  *heading = norm_angle_0_2pi(atan2f(2.0 * (q1 * q2 + q3 * q0), xx - yy - zz + ww));
  *pitch = asinf(-2.0 * (q1 * q3 - q2 * q0));
  *roll = atan2(2.0 * (q2 * q3 + q1 * q0), -xx - yy + zz + ww);
}

/**
 * Return an object with the Euler angles {heading, pitch, roll}, in radians.
 *
 * Where:
 *   - heading is from magnetic north, going west (about z-axis).
 *   - pitch is from vertical, going forward (about y-axis).
 *   - roll is from vertical, going right (about x-axis).
 *
 * Thanks to:
 *   https://github.com/PenguPilot/PenguPilot/blob/master/autopilot/service/util/quat.c#L103
 * @return {object} {heading, pitch, roll} in radians
 */
#define RAD_2_DEG (180.0f / M_PI)
void MadgwickGetEulerAnglesDegrees(float *heading, float *pitch, float *roll)
{
  MadgwickGetEulerAngles(heading, pitch, roll);

  *heading *= RAD_2_DEG;
  *pitch *= RAD_2_DEG;
  *roll *= RAD_2_DEG;
}
//====================================================================================================
// END OF CODE
//====================================================================================================
