#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from std_msgs.msg import Float32
import board
import adafruit_dht

sensor = adafruit_dht.DHT11(board.D19)
def humidity_temperature_sense():
    rospy.init_node('dht11')

    pub_temperature = rospy.Publisher('temperature', Float32, queue_size = 5)
    rate = rospy.Rate(2) # 2 Hz
    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    while not rospy.is_shutdown():
        try:
            temperature_c = sensor.temperature
            rospy.loginfo(rospy.get_caller_id() + " Measured Temperature = %.1f c", temperature_c)
            pub_temperature.publish(temperature_c)
        except RuntimeError:
            rospy.logerr(rospy.get_caller_id() + " Measured Error")
        except TypeError: # ignore type error in loginfo
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        humidity_temperature_sense()
    except rospy.ROSInterruptException:
        sensor.exit()
        rospy.loginfo(rospy.get_caller_id() + " END")
