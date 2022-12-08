#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

# GPIO Mode (Board / BCM)
GPIO.setmode(GPIO.BOARD)

key_freq = {}
key_freq["B0"] = 31
key_freq["C1"] = 33
key_freq["CS1"] = 35
key_freq["D1"] = 37
key_freq["DS1"] = 39
key_freq["E1"] = 41
key_freq["F1"] = 44
key_freq["FS1"] = 46
key_freq["G1"] = 49
key_freq["GS1"] = 52
key_freq["A1"] = 55
key_freq["AS1"] = 58
key_freq["B1"] = 62
key_freq["C2"] = 65
key_freq["CS2"] = 69
key_freq["D2"] = 73
key_freq["DS2"] = 78
key_freq["E2"] = 82
key_freq["F2"] = 87
key_freq["FS2"] = 93
key_freq["G2"] = 98
key_freq["GS2"] = 104
key_freq["A2"] = 110
key_freq["AS2"] = 117
key_freq["B2"] = 123
key_freq["C3"] = 131
key_freq["CS3"] = 139
key_freq["D3"] = 147
key_freq["DS3"] = 156
key_freq["E3"] = 165
key_freq["F3"] = 175
key_freq["FS3"] = 185
key_freq["G3"] = 196
key_freq["GS3"] = 208
key_freq["A3"] = 220
key_freq["AS3"] = 233
key_freq["B3"] = 247
key_freq["C4"] = 262
key_freq["CS4"] = 277
key_freq["D4"] = 294
key_freq["DS4"] = 311
key_freq["E4"] = 330
key_freq["F4"] = 349
key_freq["FS4"] = 370
key_freq["G4"] = 392
key_freq["GS4"] = 415
key_freq["A4"] = 440
key_freq["AS4"] = 466
key_freq["B4"] = 494
key_freq["C5"] = 523
key_freq["CS5"] = 554
key_freq["D5"] = 587
key_freq["DS5"] = 622
key_freq["E5"] = 659
key_freq["F5"] = 698
key_freq["FS5"] = 740
key_freq["G5"] = 784
key_freq["GS5"] = 831
key_freq["A5"] = 880
key_freq["AS5"] = 932
key_freq["B5"] = 988
key_freq["C6"] = 1047
key_freq["CS6"] = 1109
key_freq["D6"] = 1175
key_freq["DS6"] = 1245
key_freq["E6"] = 1319
key_freq["F6"] = 1397
key_freq["FS6"] = 1480
key_freq["G6"] = 1568
key_freq["GS6"] = 1661
key_freq["A6"] = 1760
key_freq["AS6"] = 1865
key_freq["B6"] = 1976
key_freq["C7"] = 2093
key_freq["CS7"] = 2217
key_freq["D7"] = 2349
key_freq["DS7"] = 2489
key_freq["E7"] = 2637
key_freq["F7"] = 2794
key_freq["FS7"] = 2960
key_freq["G7"] = 3136
key_freq["GS7"] = 3322
key_freq["A7"] = 3520
key_freq["AS7"] = 3729
key_freq["B7"] = 3951
key_freq["C8"] = 4186
key_freq["CS8"] = 4435
key_freq["D8"] = 4699
key_freq["DS8"] = 4978

class Buzzer:
    """A simple class for Buzzer"""
    def __init__(self, pin):
        self.pin = pin

    def setup(self):
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 440)
        self.pwm.start(0)

    def buzz(self, key):
        self.pwm.ChangeDutyCycle(50)

        self.pwm.ChangeFrequency(key_freq[key])
        rospy.loginfo(rospy.get_caller_id() + ' Buzz Key: %r, Freq: %d', key, key_freq[key])

        rospy.sleep(0.2)

        self.pwm.ChangeDutyCycle(0)

Buzzer_pin = 7
buzzer = Buzzer(Buzzer_pin)

def callback(data):
    buzzer.buzz(data.data)
 
def buzzer_control():
    rospy.init_node('buzzer')

    rospy.Subscriber('cmd_buzzer', String, callback)

    buzzer.setup()

    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")
    GPIO.cleanup()

if __name__ == '__main__':
    buzzer_control()
