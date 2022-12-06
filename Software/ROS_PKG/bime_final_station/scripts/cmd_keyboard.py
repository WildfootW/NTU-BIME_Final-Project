#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Copy from https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
# Distributed under terms of the BSD license.

import threading

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist # Default

msg = """
Reading from the keyboard and Publishing to Twist!
For Holonomic mode (strafing), hold down the shift key:
--------------------------------------
 Moving around:   | Holonomic:       |
   u    i    o    |   U    I    O    |
   j    k    l    |   J    K    L    |
   m    ,    .    |   M    <    >    |
--------------------------------------
t/b : up (+z) / down (-z)

anything else : stop

e/r : increase/decrease only linear speed 5
d/f : increase/decrease max speeds 5
c/v : increase/decrease only angular speed 5

CTRL-C to quit
"""

# x, y, z, th
moveBindings = {
        'u':( 1, 0, 0, 1),
        'i':( 1, 0, 0, 0),
        'o':( 1, 0, 0,-1),
        'j':( 0, 0, 0, 1),
        'k':( 0, 0, 0, 0),
        'l':( 0, 0, 0,-1),
        'm':(-1, 0, 0,-1),
        ',':(-1, 0, 0, 0),
        '.':(-1, 0, 0, 1),
        'U':( 1, 1, 0, 0),
        'I':( 1, 0, 0, 0),
        'O':( 1,-1, 0, 0),
        'J':( 0, 1, 0, 0),
        'K':( 0, 0, 0, 0),
        'L':( 0,-1, 0, 0),
        'M':(-1, 1, 0, 0),
        '<':(-1, 0, 0, 0),
        '>':(-1,-1, 0, 0),
        't':( 0, 0, 1, 0),
        'b':( 0, 0,-1, 0),
    }

speedBindings={
        'f':(  5,  5),
        'd':( -5, -5),
        'r':(  5,  0),
        'e':( -5,  0),
        'v':(  0,  5),
        'c':(  0, -5),
    }

class PublishThreadCmdVel(threading.Thread):
    def __init__(self, rate):
        super(PublishThreadCmdVel, self).__init__() # super() is for access methods of the base class
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('cmd_keyboard')

    speed = rospy.get_param("~speed", 10.0)
    turn = rospy.get_param("~turn", 10.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", None) # None for disable timeout / "0" indicate immediately timeout
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread_vel = PublishThreadCmdVel(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0 # for resent the message

    try:
        pub_thread_vel.update(x, y, z, th, speed, turn)

        rospy.loginfo(msg)
        rospy.loginfo("currently: speed %.3f\tturn %.3f ", speed, turn)
        while True:
            key = getKey(settings, key_timeout)

            if status == 10:
                rospy.loginfo(msg)
                status = 0

            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                y  = moveBindings[key][1]
                z  = moveBindings[key][2]
                th = moveBindings[key][3]

                pub_thread_vel.update(x, y, z, th, speed, turn)
            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn  = turn  + speedBindings[key][1]

                speed = min(speed, 100)
                turn  = min(turn , 100)
                speed = max(speed,   0)
                turn  = max(turn ,   0)

                pub_thread_vel.update(x, y, z, th, speed, turn)
                rospy.loginfo("currently: speed %.3f\tturn %.3f ", speed, turn)
                status = status + 1
            else:
                # Skip updating cmd_vel if key timeout and robot already stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break


    except Exception as e:
        rospy.loginfo(e)

    finally:
        pub_thread_vel.stop()
        restoreTerminalSettings(settings)
