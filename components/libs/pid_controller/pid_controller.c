#include "pid_controller.h"

// int16_t motor_kp = 300; //电机转速PID-P
// int16_t motor_ki = 0;   //电机转速PID-I
// int16_t motor_kd = 200; //电机转速PID-D

#define PID_INTEGRAL_UP 200 //积分上限

int16_t pid_update(pid_controller_t *pid_controller, int16_t target, int16_t current)
{
    pid_controller->pid_calcute.bias = target - current;
    pid_controller->pid_calcute.bias_sum += pid_controller->pid_calcute.bias;
    pid_controller->pid_calcute.dbias = pid_controller->pid_calcute.bias_last - pid_controller->pid_calcute.bias_pre;

    pid_controller->pid_calcute.bias_pre = pid_controller->pid_calcute.bias_last;
    pid_controller->pid_calcute.bias_last = pid_controller->pid_calcute.bias;

    if (pid_controller->pid_calcute.bias_sum > PID_INTEGRAL_UP)
        pid_controller->pid_calcute.bias_sum = PID_INTEGRAL_UP;
    if (pid_controller->pid_calcute.bias_sum < -PID_INTEGRAL_UP)
        pid_controller->pid_calcute.bias_sum = -PID_INTEGRAL_UP;

    pid_controller->output = pid_controller->kp * pid_controller->pid_calcute.bias + pid_controller->ki * pid_controller->pid_calcute.bias_sum + pid_controller->kd * pid_controller->pid_calcute.dbias;
    /*控制最终输出的PID大小，根据PWM为13位定制2^13*/
    if (pid_controller->output > pid_controller->limit)
        pid_controller->output = pid_controller->limit;
    if (pid_controller->output < -pid_controller->limit)
        pid_controller->output = -pid_controller->limit;
    return 0;
}
