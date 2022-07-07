# FishBot机器人运动控制板子-嵌入式驱动原理图PCB



# 构建
```
docker run -it  --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` espressif/idf:release-v4.4 idf.py build
```

## 烧写
```
docker run -it  --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` espressif/idf:release-v4.4 idf.py flash
```

# 嵌入式驱动结构

目前组件目录

- fishbot
    - general
        - led
        - motors
            - motor
            - rotary_encoder
    - i2c_devices
        - i2c_master
        - oled
- libs
    - pid_controller


关于FishBot配置的一些思考

主要分清楚，哪些我们是动态配置，哪些我们写死即可。

电机数量、编码器/mm(电机的减速比、轮胎直径)、PID、wifi帐号和密码、Server地址端口号、通讯模式（USB(UART)/WIFI(UDP)）

可以写死的

电机引脚配置、MPU6050引脚配置、LED引脚配置、OLED引脚配置