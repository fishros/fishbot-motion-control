# FishBot机器人运动控制板子-嵌入式驱动原理图PCB


## TODO&DONE
- [x] 电机驱动\编码器计算 
- [x] OLED驱动与显示
- [x] WIFI模块
- [ ] 电机PID控制计算
- [ ] MPU6050任务数据获取及姿态推算
- [ ] UDP协议收发任务实现
- [ ] Protocol数据接受及处理实现
- [ ] Protocol数据发送处理及实现
- [ ] NVS掉电存储模块（WIFI配置、PID配置）
- [ ] 按键驱动实现（模式切换功能）
- [ ] 日志模块搭建（支持输出到UDPServer）
- [ ] 制定新版本协议（支持模式配置、WIFI配置，增加PID更多参数，冗余电机数量）
- [ ] 新版本协议WIFI模块实现
- [ ] 新版本协议PID模块实现
- [ ] 新版本协议电机模块实现


## 开发环境搭建

### 构建
```
docker run -it  --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` espressif/idf:release-v4.4 idf.py build
```

### 烧写
```
docker run -it  --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` espressif/idf:release-v4.4 idf.py flash
```


## 工程介绍

### 嵌入式驱动结构

目前组件目录

```
├── config
│   └── include
│       ├── config.h
│       └── task_config.h
├── drivers
│   ├── general
│   │   ├── key
│   │   ├── led
│   │   │   ├── include
│   │   │   │   └── led.h
│   │   │   └── led_esp32.c
│   │   ├── motors
│   │   │   └── motor
│   │   │       ├── include
│   │   │       │   └── motor.h
│   │   │       └── motor.c
│   │   └── wifi
│   │       ├── include
│   │       │   └── wifi.h
│   │       └── wifi.c
│   └── i2c_devices
│       ├── mpu6050
│       │   ├── include
│       │   │   ├── mpu6050.h
│       │   │   └── mpu6050_registers.h
│       │   └── mpu6050.c
│       └── oled
│           ├── include
│           │   └── oled.h
│           └── oled.c
├── fishbot
│   ├── fishbot.c
│   ├── fishbot_config.c
│   └── include
│       ├── fishbot_config.h
│       └── fishbot.h
├── libs
│   ├── esp32_i2c_rw
│   │   ├── esp32_i2c_rw.c
│   │   └── include
│   │       └── esp32_i2c_rw.h
│   ├── pid_controller
│   │   ├── include
│   │   │   └── pid_controller.h
│   │   └── pid_controller.c
│   └── rotary_encoder
│       ├── include
│       │   └── rotary_encoder.h
│       └── src
│           └── rotary_encoder_pcnt_ec11.c
└── protocol
    ├── proto
    │   ├── include
    │   │   └── protocol.h
    │   └── protocol.c
    ├── proto_utils
    │   ├── include
    │   │   ├── proto_utils.h
    │   │   └── proto_v1.0.0.20220621.h
    │   ├── proto_utils.c
    │   └── proto_v1.0.0.20220621.c
    ├── uart_protocol
    │   ├── include
    │   │   └── uart_protocol.h
    │   └── uart_protocol.c
    └── udp_client_protocol
        ├── include
        │   └── udp_client_protocol.h
        └── udp_client.c
```
