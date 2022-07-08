# -*- coding: utf-8 -*-
from base_control import BaseControl
import rclpy



if __name__ == '__main__':
    rclpy.init()
    base_control_node =  BaseControl()
    rclpy.spin(base_control_node)
    rclpy.shutdown()