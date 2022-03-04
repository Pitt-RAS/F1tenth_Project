#!/usr/bin/env python2
from time import time

import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
import math

import numpy as np

PUSH_TO_POWER = 100
TURN_TO_TWIST = 100


class JoystickDriver:
    def __init__(self):
        self.joy_drive_pub = rospy.Publisher('/joy_drive', AckermannDriveStamped, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
    
    def joy_callback(self, joy_data):
        message = self.create_AckermannDriveStamped(joy_data)
        message.drive.steering_angle = self.get_steering_angle(joy_data)
        message.drive.speed = self.get_speed(joy_data)
        self.joy_drive_pub.publish(message)

    def get_speed(self, joy_data):
        return joy_data.axes[4] * PUSH_TO_POWER

    def get_steering_angle(self, joy_data):
        x = joy_data.axes[0]
        y = joy_data.axes[1]
        #return find_angle(x, y) * RAD_TO_TWIST
        return x * TURN_TO_TWIST

    def create_AckermannDriveStamped(self, joy_data):
        message = AckermannDriveStamped()
        message.header = joy_data.header
        return message

def find_angle(x, y):
    h = math.hypot(x, y)
    if h==0:
        return 0
    angle = math.asin(x/h)
    return angle

def main():
    rospy.init_node('joystick_driver')
    rate = rospy.Rate(10)
    JoystickDriver_obj = JoystickDriver()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass