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


class HC_SR04:
    """A simple class for HC-SR04"""
    def __init__(self, trigger, echo):
        self.trigger_pin = trigger
        self.echo_pin    = echo

    def setup(self):
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
    trigger_pin = rospy.get_param("~trigger_pin", 7)
    echo_pin    = rospy.get_param("~echo_pin",   11)
 
    sensor = HC_SR04(trigger_pin, echo_pin)

    rospy.init_node('sonic_distance', anonymous = True)

    pub_buzzer = rospy.Publisher('cmd_buzzer', Bool, queue_size = 1)
#    pub_distance = rospy.Publisher('distance', Bool, queue_size = 1)

    sensor.setup()
    rate = rospy.Rate(2) # 2 Hz
    rospy.loginfo(rospy.get_caller_id() + " Setup Trigger Pin: %d, Echo Pin: %d.", trigger_pin, echo_pin)

    while not rospy.is_shutdown():
        distance = sensor.measure_distance()
        rospy.loginfo(rospy.get_caller_id() + " Measured Distance = %.1f cm", distance)
        if distance < 10:
            pub_buzzer.publish(True)
        rate.sleep()

if __name__ == '__main__':
    try:
        ultrasonic_distance_sense()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        rospy.loginfo(rospy.get_caller_id() + " END")