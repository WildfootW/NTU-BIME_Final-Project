#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

# GPIO Mode (Board / BCM)
GPIO.setmode(GPIO.BOARD)

class Buzzer:
    """A simple class for Buzzer"""
    def __init__(self, pin):
        self.pin = pin

    def setup(self):
        GPIO.setup(self.pin, GPIO.OUT)

    def buzz(self, bee):
        GPIO.output(self.pin, GPIO.HIGH)

        rospy.sleep(0.2)

        GPIO.output(self.pin, GPIO.LOW)

Buzzer_pin = 7
buzzer = Buzzer(Buzzer_pin)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received CMD')
    buzzer.buzz(data.data)
 
def buzzer_control():
    rospy.init_node('buzzer')

    rospy.Subscriber('cmd_buzzer', Bool, callback)

    buzzer.setup()

    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")
    GPIO.cleanup()

if __name__ == '__main__':
    buzzer_control()
