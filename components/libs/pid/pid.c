#include "pid.h"

// 全局变量
// int16_t target_spped_left = 0;
// int16_t target_spped_right = 0 ;
// float tick_per_merter = 7.8390;

int16_t motor_kp = 300; //电机转速PID-P
int16_t motor_ki = 0;   //电机转速PID-I
int16_t motor_kd = 200; //电机转速PID-D

#define PID_INTEGRAL_UP 200 //积分上限

uint16_t pid_update(pid_config_t *pid_config, int16_t target, int16_t current)
{
    pid_config->pid_calcute->bias = target - current;
    pid_config->pid_calcute->bias_sum += pid_config->pid_calcute->bias;
    pid_config->pid_calcute->dbias = pid_config->pid_calcute->bias_last - pid_config->pid_calcute->bias_pre;

    pid_config->pid_calcute->bias_pre = pid_config->pid_calcute->bias_last;
    pid_config->pid_calcute->bias_last = pid_config->pid_calcute->bias;

    if (pid_config->pid_calcute->bias_sum > PID_INTEGRAL_UP)
        pid_config->pid_calcute->bias_sum = PID_INTEGRAL_UP;
    if (pid_config->pid_calcute->bias_sum < -PID_INTEGRAL_UP)
        pid_config->pid_calcute->bias_sum = -PID_INTEGRAL_UP;

    // ESP_LOGI(TAG, "bias:%d bias_sum:%d dbias:%d",pid->bias,pid->bias_sum,pid->dbias);

    int16_t output = motor_kp * pid_config->pid_calcute->bias + motor_ki * pid_config->pid_calcute->bias_sum + motor_kd * pid_config->pid_calcute->dbias;
    /*控制最终输出的PID大小，根据PWM为13位定制2^13*/
    if (output > 8192)
        output = 8192;
    if (output < -8192)
        output = -8192;
    return 0;
}
