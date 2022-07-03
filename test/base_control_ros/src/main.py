'''
作者: 小鱼
公众号: 鱼香ROS
QQ交流群: 2642868461
描述: file content
'''
# -*- coding: utf-8 -*-
from base_control import BaseControl
import rospy


if __name__ == '__main__':
    rospy.init_node("base_control")
    base_control_node =  BaseControl()
    rospy.spin()