#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received Linear: x=%.2f, y=%.2f, z=%.2f | Angular: x=%.2f, y=%.2f, z=%.2f', data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z)

def wheel_control():
    rospy.init_node('wheel_control')

    rospy.Subscriber('cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    wheel_control()
