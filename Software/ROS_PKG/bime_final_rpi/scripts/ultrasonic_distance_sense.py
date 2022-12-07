#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from std_msgs.msg import Bool
#from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time

# GPIO Mode (Board / BCM)
GPIO.setmode(GPIO.BOARD)

# set GPIO Pins
TRIGGER_PIN = 7
ECHO_PIN = 11
 
class HC_SR04:
    """A simple class for HC-SR04"""
    def __init__(self, trigger, echo):
        self.trigger_pin = trigger
        self.echo_pin    = echo

    def setup(self):
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
 
    def measure_distance(self):
        GPIO.output(self.trigger_pin, False)
        time.sleep(0.000002)
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)
     
        start_time = time.time()
        stop_time = time.time()
     
        # save start_time
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()
     
        # save time of arrival
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()
     
        # time difference between start and arrival
        time_elapsed = stop_time - start_time
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (time_elapsed * 34300) / 2
     
        return distance
 
def ultrasonic_distance_sense():
    rospy.init_node('sonic_distance', anonymous = True)

    sensor = HC_SR04(TRIGGER_PIN, ECHO_PIN)
    pub_buzzer = rospy.Publisher('cmd_buzzer', Bool, queue_size = 1)
#    pub_distance = rospy.Publisher('distance', Bool, queue_size = 1)

    rospy.loginfo(rospy.get_caller_id() + " Measured Distance = %.1f cm", distance)
    led.setup()

    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")
    GPIO.cleanup()

if __name__ == '__main__':
    ultrasonic_distance_sense()

 

