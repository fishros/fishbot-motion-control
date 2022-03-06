/*
 * @作者: 小鱼
 * @公众号: 鱼香ROS
 * @QQ交流群: 2642868461
 * @描述: 电机控制文件
 */
#include "motor.h"
#include "udp_client.h"

#define TAG "fishcar-motor"

// 全局变量
int16_t target_spped_left = 0;
int16_t target_spped_right = 0 ;
float tick_per_merter = 7.8390;

int16_t motor_kp=300;  //电机转速PID-P
int16_t motor_ki=0;    //电机转速PID-I
int16_t motor_kd=200;  //电机转速PID-D

#define PID_SCALE  0.01f  //PID缩放系数
#define PID_INTEGRAL_UP 200  //积分上限


// 引脚的宏定义
#define GPIO_OUTPUT_IO_LEFT_0    22
#define GPIO_OUTPUT_IO_LEFT_1    23
#define GPIO_OUTPUT_IO_RIGHT_0   13
#define GPIO_OUTPUT_IO_RIGHT_1   12

#define GPIO_OUTPUT_PWM_LEFT     18
#define GPIO_OUTPUT_PWM_RIGHT    19

#define GPIO_OUTPUT_ENCODER_LEFT_0    32
#define GPIO_OUTPUT_ENCODER_LEFT_1    33
#define GPIO_OUTPUT_ENCODER_RIGHT_0   26
#define GPIO_OUTPUT_ENCODER_RIGHT_1   25


/**
 * @description: 创建编码器pcnt
 * @param {uint32_t} pcnt_unit 编号
 * @param {uint8_t} channela_num 通道A引脚号
 * @param {uint8_t} channelb_num 通道B引脚号
 * @return {rotary_encoder_t * } 编码器
 */

rotary_encoder_t * create_rotary(uint32_t pcnt_unit,uint8_t channela_num,uint8_t channelb_num)
{
    rotary_encoder_t *encoder = NULL;
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, channela_num, channelb_num);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));
    ESP_ERROR_CHECK(encoder->start(encoder));
    return encoder;
}

/**
 * @description: 电机速度pwm初始化
 * @param {void}
 * @return {void}
 */
void pwm_init()
{
      ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel[2] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = GPIO_OUTPUT_PWM_LEFT,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = GPIO_OUTPUT_PWM_RIGHT,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        }
    };
    //左边轮子pwm和右边轮子pwm初始化
    ledc_channel_config(&ledc_channel[0]);
    ledc_channel_config(&ledc_channel[1]);
}


/**
 * @description: 前进后退IO初始化
 * @param {*}
 * @return {*}
 */
void io_init()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask =  (1ULL<<GPIO_OUTPUT_IO_LEFT_0) | (1ULL<<GPIO_OUTPUT_IO_LEFT_1) | (1ULL<<GPIO_OUTPUT_IO_RIGHT_0) | (1ULL<<GPIO_OUTPUT_IO_RIGHT_1);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_IO_LEFT_0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_LEFT_1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_RIGHT_0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_RIGHT_1, 0);
}






void pid_update(my_pid_t* pid,int16_t target, int16_t current)
{
    pid->bias = target - current;
    pid->bias_sum += pid->bias;
    pid->dbias = pid->bias_last-pid->bias_pre;

    pid->bias_pre = pid->bias_last;
    pid->bias_last = pid->bias;

    if(pid->bias_sum>PID_INTEGRAL_UP)pid->bias_sum = PID_INTEGRAL_UP;
	if(pid->bias_sum<-PID_INTEGRAL_UP)pid->bias_sum = -PID_INTEGRAL_UP;

    // ESP_LOGI(TAG, "bias:%d bias_sum:%d dbias:%d",pid->bias,pid->bias_sum,pid->dbias);

    pid->output += (motor_kp*pid->bias
                        +motor_ki*pid->bias_sum
                        +motor_kd*pid->dbias)
                        *PID_SCALE;
    /*控制最终输出的PID大小，根据PWM为13位定制2^13*/
    if(pid->output > 8192)  pid->output = 8192;
    if(pid->output < -8192) pid->output = -8192;
}


static void rotary_task(void *arg)
{
    
    //初始化PWM
    pwm_init();
    
    //方向控制IO初始化
    io_init();
    
    //编码器初始化
    rotary_encoder_t *encoder_left = create_rotary(0, GPIO_OUTPUT_ENCODER_LEFT_0, GPIO_OUTPUT_ENCODER_LEFT_1);
    rotary_encoder_t *encoder_right = create_rotary(1,GPIO_OUTPUT_ENCODER_RIGHT_0,GPIO_OUTPUT_ENCODER_RIGHT_1);

    int32_t tick_left = encoder_left->get_counter_value(encoder_left);
    int32_t tick_right = encoder_right->get_counter_value(encoder_right);
    
    // pid 结构体初始化
    my_pid_t pid_left,pid_right;

    // // 速度以及速度计算所需变量
    float spped_left, spped_right = 0 ;
    int32_t tick_count = xTaskGetTickCount();
    int16_t time_spend = 0;

    motor frame = {
        .START=0x7D,
        .TARGET = 0x01,
        .CODE = 0X01,
        .data_len = 0x04,
        .END = 0x7E,
    };

    while (1) {
        //花费的时间
        time_spend = (xTaskGetTickCount()-tick_count);
        //计算左右轮速度
        spped_left = encoder_left->get_counter_value(encoder_left) - tick_left;
        spped_left = (float)spped_left/tick_per_merter/time_spend*100;
        spped_right = encoder_right->get_counter_value(encoder_right) - tick_right;
        spped_right = (float)spped_right/tick_per_merter/time_spend*100;

        // ESP_LOGI(TAG,"tick_left:%d spped_left:%f ", encoder_left->get_counter_value(encoder_left) - tick_left,spped_left);
        // 更新tick_count 和 tick
        tick_count = xTaskGetTickCount();
        tick_left = encoder_left->get_counter_value(encoder_left);
        tick_right = encoder_right->get_counter_value(encoder_right);
        


        
        pid_update(&pid_left,target_spped_left,(int)spped_left);
        if(pid_left.output>=0){
            gpio_set_level(GPIO_OUTPUT_IO_LEFT_0, 1);
            gpio_set_level(GPIO_OUTPUT_IO_LEFT_1, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pid_left.output);
        }else{
            gpio_set_level(GPIO_OUTPUT_IO_LEFT_0, 0);
            gpio_set_level(GPIO_OUTPUT_IO_LEFT_1, 1);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, -pid_left.output);
        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        

        pid_update(&pid_right,target_spped_right,(int)spped_right);
        if(pid_right.output>=0){
            gpio_set_level(GPIO_OUTPUT_IO_RIGHT_0, 1);
            gpio_set_level(GPIO_OUTPUT_IO_RIGHT_1, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, pid_right.output);
        }else{
            gpio_set_level(GPIO_OUTPUT_IO_RIGHT_0, 0);
            gpio_set_level(GPIO_OUTPUT_IO_RIGHT_1, 1);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, -pid_right.output);
        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

        ESP_LOGI(TAG,"speed_left:%f->%d  duty_left:%d", spped_left,target_spped_left,pid_left.output);
        ESP_LOGI(TAG,"speed_right:%f->%d  duty_right:%d", spped_right,target_spped_right,pid_right.output);

        // 更新速度并发送
        frame.spped_left = spped_left;
        frame.spped_right = spped_right;
        frame.sum =  calc_checksum((char *)&frame+4,frame.data_len);
        send_data((char *)&frame,sizeof(frame));

        vTaskDelay(pdMS_TO_TICKS(33));
    }
    vTaskDelete(NULL);
}

/**
 * @description: 电机任务初始化
 * @param {*}
 * @return {*}
 */
void my_motor_init()
{    
    xTaskCreate(rotary_task, "rotary_task", 4*1024, NULL, 5, NULL);
}