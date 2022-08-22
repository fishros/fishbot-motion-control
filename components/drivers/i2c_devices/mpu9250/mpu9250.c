/**
 * @brief MPu9250 IMU驱动
 * @author cuizhongren45 (1326986768@qq.com)
 * @version V1.0.0
 * @date 2022-08-22
 * @copyright 版权所有：FishBot Open Source Organization
 * 主要参考：https://github.com/psiphi75/esp-mpu9250
 */
#include "mpu9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PI (3.14159265358979323846f)
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define Acc_Gain 16384.0
#define Gyro_Gain 131.0
#define GYRO_MEAS_DRIFT (PI * (1.0f / 180.0f))

#define Kp      15.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki       0.01f   // integral gain governs rate of convergence of gyroscope biases
#define halfT    0.02f   // integral gain governs rate of convergence of gyroscope biases
#define Rad_to_Angle						57.324841    //弧度到角度

volatile float exInt=0.0f, eyInt=0.0f, ezInt=0.0f;  // 误差积分
volatile float q[4]; //　四元数暂存变量
static const char *TAG = "mpu9250";
static bool initialised = false;

int filter_time = 1000;
float accoffset[3],gyrooffset[3];
int16_t accel_[3],gyro_[3];
float yaw9250, pitch9250, roll9250;
int64_t now_time9250 = 0, prev_time9250 = 0;
float pitch9250,roll9250,yaw9250;
float accX9250 ,accY9250 ,accZ9250 ,gyroX9250 ,gyroY9250 ,gyroZ9250;
float angleAccX9250 ,angleAccY9250;
float interval9250 ;
float angleGyroX9250,angleGyroY9250 ,angleGyroZ9250;
float angleX9250 ,angleY9250 ,angleZ9250 ;
float axa_x,axa_y,axa_z;
float axa_time = 100;
float _hxb, _hyb, _hzb;
float magmax[3] = {-100, -100, -100};
float magmin[3] = {100 , 100, 100};
float magsum[3],magoffset[3],maggain[2];
float magval[3];


volatile float sampleFreq = 50;                            // 2 * proportional gain (Kp)
volatile float beta = 0.8;                                 // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

calibration_t cal = {};


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

    mpu9250_set_full_scale_accel_range(MPU9250_ACCEL_FS_2);
    vTaskDelay(10 / portTICK_RATE_MS);

    mpu9250_set_sleep_enabled(0);
    vTaskDelay(10 / portTICK_RATE_MS);
    ESP_LOGE(TAG, "END of MPU9250 initialization");
    

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



void ak8963_get_sensitivity_adjustment_values()
{
  // Need to set to Fuse mode to get valid values from this.
  uint8_t current_mode;
  ak8963_get_cntl(&current_mode);

  ak8963_set_cntl(AK8963_CNTL_MODE_FUSE_ROM_ACCESS);

  vTaskDelay(20 / portTICK_RATE_MS);

  // uint8_t xi, yi, zi;
  // for(int i = 0;i<axa_time;i++)
  // {
  // esp32_i2c_read_byte( AK8963_ADDRESS, AK8963_ASAX, &xi);
  // esp32_i2c_read_byte(AK8963_ADDRESS, AK8963_ASAY, &yi);
  // esp32_i2c_read_byte(AK8963_ADDRESS, AK8963_ASAZ, &zi);

  // // Get the ASA* values
  // axa_x += (float)xi / 128.0 ;
  // axa_y += (float)yi / 128.0 ;
  // axa_z += (float)zi / 128.0 ;
  // }
  // axa_x = axa_x /axa_time;
  // axa_y = axa_y /axa_time;
  // axa_z = axa_z /axa_time;


  ak8963_set_cntl(current_mode);
}


bool ak8963_init()
{
  uint8_t id;
  esp32_i2c_read_byte(AK8963_ADDRESS,AK8963_WHO_AM_I,&id);

  if(id & AK8963_WHO_AM_I_RESPONSE)  //AK8963_WHO_AM_I_RESPONSE
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
    //ak8963磁力计初始化
    ak8963_init();
    ESP_LOGI(TAG, "Magnetometer enabled");
    return true;
  }
  else
  {
    ESP_LOGE(TAG, "Can't turn on RA_INT_PIN_CFG.");
    return false;
  }

}


bool mpu9250_task_init(void)
{
    mpu9250_init(&cal);
    return true;
}



void cal_offset()
{
  vector_t va, vg, vm;
      for(int i= 0;i<filter_time;i++)
    {
      get_accel_gyro_mag(&va, &vg, &vm);
      accoffset[0] += ((float)va.x) / Acc_Gain;
      accoffset[1] += ((float)va.y) / Acc_Gain;
      accoffset[2] += ((float)va.z) / Acc_Gain;
      gyrooffset[0] += ((float)vg.x) / Gyro_Gain;
      gyrooffset[1] += ((float)vg.y) / Gyro_Gain;
      gyrooffset[2] += ((float)vg.z) / Gyro_Gain;
      ESP_LOGI(TAG,"Calculate bias,  Rest %d",filter_time-i);
    }
      accoffset[0] = (accoffset[0]) / filter_time;
      accoffset[1] = (accoffset[1]) / filter_time;
      accoffset[2] = (accoffset[2]) / filter_time;
      gyrooffset[0] = (gyrooffset[0]) / filter_time;
      gyrooffset[1] = (gyrooffset[1]) / filter_time;
      gyrooffset[2] = (gyrooffset[2]) / filter_time;

}


void ak8963_caloffset()
{
    vector_t va, vg, vm;
    for(int i = 0;i < 1000; i++)
    {
      get_accel_gyro_mag(&va, &vg, &vm);
      if((abs(vm.x) < 400)  && (abs(vm.y) < 400) && (abs(vm.z) < 400))
      {
        magmax[0] = _MAX(vm.x,magmax[0]);
        magmax[1] = _MAX(vm.y,magmax[1]);
        magmax[2] = _MAX(vm.z,magmax[2]);

        magmin[0] = _MIN(vm.x,magmin[0]);
        magmin[1] = _MIN(vm.y,magmin[1]);
        magmin[2] = _MIN(vm.z,magmin[2]);
      }
    }

    magoffset[0] = (magmax[0] + magmin[0]) * 0.5f;
    magoffset[1] = (magmax[1] + magmin[1]) * 0.5f;
    magoffset[2] = (magmax[2] + magmin[2]) * 0.5f;

    magsum[0] = magmax[0] - magmin[0];
    magsum[1] = magmax[1] - magmin[1];
    magsum[2] = magmax[2] - magmin[2];   

    maggain[0] = magsum[0] / magsum[1];
    maggain[1] = magsum[0] / magsum[2];

}

float _MAX(float x, float y)
{
  if(x>=y) return x;
  else return y;
}

float _MIN(float x, float y)
{
  if(x <= y) return x;
  else return y;
}

void get_mpu9250_euler_angle(void *param)
{
  cal_offset();  
  ak8963_caloffset();
  while(true)
  { 

    now_time9250 = esp_timer_get_time();
    interval9250 = (float)(now_time9250 - prev_time9250)/1000000.0f;
    vector_t va, vg, vm;
    //获取原始数据
    get_accel_gyro_mag(&va, &vg, &vm);

    accX9250 = ((float)-va.x) / Acc_Gain;
    accY9250 = ((float)-va.y) / Acc_Gain;
    accZ9250 = ((float)-va.z) / Acc_Gain;

    gyroX9250 = (((float)vg.x) / Gyro_Gain)-gyrooffset[0];
    gyroY9250 = (((float)vg.y) / Gyro_Gain)-gyrooffset[1];
    gyroZ9250 = (((float)vg.z) / Gyro_Gain)-gyrooffset[2];

    angleAccX9250 = atan2(accY9250, sqrt(accZ9250 * accZ9250 + accX9250 * accX9250)) * 360 / 2.0 / 3.14159265358979323846f;
    angleAccY9250 = atan2(accX9250, sqrt(accZ9250 * accZ9250 + accY9250 * accY9250)) * 360 / -2.0 / 3.14159265358979323846f;

    angleGyroX9250 += gyroX9250 * interval9250;
    angleGyroY9250 += gyroY9250 * interval9250;
    angleGyroZ9250 += gyroZ9250 * interval9250;

    pitch9250 = (0.98 * (angleX9250 + gyroX9250 * interval9250)) + (0.02 * angleAccX9250);
    roll9250 = (0.98 * (angleY9250 + gyroY9250 * interval9250)) + (0.02 * angleAccY9250);
    yaw9250= angleGyroZ9250;

    while(yaw9250 < 0 ) yaw9250 += 360;
    while(yaw9250 > 360 ) yaw9250 -=360;

    magval[0] = vm.x;// - magoffset[0];
    magval[1] = vm.y;//- magoffset[1];
    magval[2] = vm.z;// - magoffset[2];

    // //应用AHRS算法
    	if((magval[0] == -1) && (magval[1] == -1) && (magval[2] == -1))
      {
        ;
      }
      else
      {
        IMU_update(gyroX9250 * GYRO_MEAS_DRIFT, gyroY9250 * GYRO_MEAS_DRIFT, gyroZ9250 * GYRO_MEAS_DRIFT,
                                va.x, va.y, va.z,
                                magval[0],magval[1], magval[2]);
      }
    ESP_LOGI(TAG, "yaw: %2.3f, pitch: %2.3f, roll: %2.3f", yaw9250, pitch9250, roll9250);
    //获得温度，可以检验mpu9250的读写是否正常
    // float temp;
    // //get_temperature_celsius(&temp);  
    // temp = (atan2(vm.y, vm.x)) * 57.3; 
    // ESP_LOGI(TAG, "Temp %2.3f",temp);

    vTaskDelay(20 / portTICK_RATE_MS);
    prev_time9250 = now_time9250;
  
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

    
    v->x = xi; //* axa_x ;
    v->y = yi; //* axa_y ;
    v->z = zi; //* axa_z ;

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
  vTaskDelay(2 / portTICK_RATE_MS);
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

  v->x = xi;
  v->y = yi;
  v->z = zi;
}

void align_gryo(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);


  v->x = xi;
  v->y = yi;
  v->z = zi;
}


void get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm)
{
  get_accel_gyro(va, vg);
  ak8963_get_mag(vm);
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

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float tempq0,tempq1,tempq2,tempq3;

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加计的三维向量转成单位向量。

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);//这里为了减少运算量  因为q1、q2、q3、q4平方和是等于1 的
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
      
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
	

	
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  

  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);


	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
			exInt = exInt + ex * Ki * halfT;
			eyInt = eyInt + ey * Ki * halfT;
			ezInt = ezInt + ez * Ki * halfT;

			
			gx = gx + Kp*ex + exInt;// 用叉积误差来做PI修正陀螺零偏
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;

  }

  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // 四元数规范化
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;


  
    
	yaw9250 = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)/ GYRO_MEAS_DRIFT; // yaw
	pitch9250 = -asin(-2 * q1 * q3 + 2 * q0 * q2)/ GYRO_MEAS_DRIFT; // pitch
	roll9250 = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)/ GYRO_MEAS_DRIFT; // roll
	
}

void mpu9250_task(void)
{
    
    xTaskCreate(get_mpu9250_euler_angle, "get_mpu9250_euler_angle", 8 * 1024, NULL, 5, NULL);
}

