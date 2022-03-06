/*
 * @作者: 小鱼
 * @公众号: 鱼香ROS
 * @QQ交流群: 2642868461
 * @描述: file content
 */
#include "motor.h"
#include "mwifi.h"
#include "udp_client.h"
#include "imu_wit.h"

static const char *TAG = "fishcar";


void app_main(void)
{
    wifi_init();
    wifi_set_as_sta("m","88888888");
    // wifi_set_as_sta("InforeRobot","12341234ok");
    my_udp_init();
    my_motor_init();
    my_imu_init();
}
