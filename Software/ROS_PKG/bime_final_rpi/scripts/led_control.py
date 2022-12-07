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

# Set GPIO Pins
LED_pin = 22

class LED:
    """A simple class for LED"""
    def __init__(self, pin):
        self.pin = pin

    def setup(self):
        GPIO.setup(self.pin, GPIO.OUT)

    def switch(self, bright):
        if bright:
            GPIO.output(self.pin, GPIO.HIGH)
            rospy.loginfo(rospy.get_caller_id() + ' Turn On')
        else:
            GPIO.output(self.pin, GPIO.LOW)
            rospy.loginfo(rospy.get_caller_id() + ' Turn Off')

led = LED(LED_pin)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received CMD')
    led.switch(data.data)
 
def led_control():
    rospy.init_node('led')

    rospy.Subscriber('cmd_led', Bool, callback)

    led.setup()

    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")
    GPIO.cleanup()

if __name__ == '__main__':
    led_control()
