# ESP32 MPU6050

Code provided in this repository is a MPU6050 library for ESP 32.

## Table of Contents

- [Components](#components)
  - [ESP 32](#esp-32)
  - [ESP-IDF](#esp-idf)
  - [MPU-6050](#mpu-6050)
- [Quick Start](#quick-start)
- [Acknowledgments](#acknowledgments)

## Components

To make this code work, you need the following components:

* This repository.
* [ESP 32](https://espressif.com/en/products/hardware/esp32/overview) module.
* [ESP-IDF](https://github.com/espressif/esp-idf).
* [MPU-6050](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/) module.

### ESP 32

Any ESP 32 module should work.

### ESP-IDF

Configure your PC according to [ESP 32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/?badge=latest?badge=latest). Windows, Linux and Mac OS are supported.

### MPU-6050

This example has been tested with a MPU-6050. 

The MPU-6000 should work aswell.

## Quick Start

This directory is an ESP-IDF component. Clone it (or add it as a submodule) into the component directory of the project.

To use this library you also need the [esp32-i2c_rw](https://github.com/gabrielbvicari/esp32-i2c_rw) library. Clone it in the same folder as you did with the MPU-6050.

To modify or understand the code please read the [MPU-6050 Datasheet](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) guide and the [ESP 32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/?badge=latest?badge=latest).

## Acknowledgments

This application is using code based on:

* Implementation in C++ for ESP 32 by [Jeff Rowberg](https://www.i2cdevlib.com).