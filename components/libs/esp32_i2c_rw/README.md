# ESP32 I2C READ/WRITE

Code provided in this repository implements read/write I2C functions for ESP 32.

## Table of Contents

- [Components](#components)
  - [ESP 32](#esp-32)
  - [ESP-IDF](#esp-idf)
- [Quick Start](#quick-start)
- [Acknowledgments](#acknowledgments)

## Components

To make this code work, you need the following components:

* This repository.
* [ESP 32](https://espressif.com/en/products/hardware/esp32/overview) module.
* [ESP-IDF](https://github.com/espressif/esp-idf).

### ESP 32

Any ESP 32 module should work.

### ESP-IDF

Configure your PC according to [ESP 32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/?badge=latest?badge=latest). Windows, Linux and Mac OS are supported.

## Quick Start

To understand the I2C protocol, follow the [ESP 32 I2C Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/i2c.html?highlight=i2c) guide.

This directory is an ESP-IDF component. Clone it (or add it as a submodule) into the component directory of the project.

## Acknowledgments

This application is using code based on:

* Implementation in C++ for ESP 32 by [Jeff Rowberg](https://www.i2cdevlib.com).