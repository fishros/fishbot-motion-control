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
