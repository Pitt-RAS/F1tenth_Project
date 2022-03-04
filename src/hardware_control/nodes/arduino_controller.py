#!/usr/bin/env python2
import time

import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
import math

import numpy as np
import serial

bitrate = 115200
port = '/dev/ttyACM0'

MAX_ANGLE = 130
MIN_ANGLE = 60

MAX_SPEED = 30

KILL_ANGLE = 95
KILL_SPEED = 90

TIMER_PERIOD = 1

arduino = serial.Serial(port, bitrate)
arduino.timeout=1

class ArduinoController:
    def __init__(self):

        self.joy_drive_sub = rospy.Subscriber('joy_drive', AckermannDriveStamped, self.joy_drive_callback)
        self.arduino_timer_sub = rospy.Subscriber('/arduino_timer', std_msgs.msg.Empty, self.timer_callback)
        self.joy_drive_data = AckermannDriveStamped()
    
    def joy_drive_callback(self, joy_drive_data):
        self.joy_drive_data = joy_drive_data

    def timer_callback(self, empty_data):
        joy_drive_data = self.joy_drive_data
        twist = joy_drive_data.drive.steering_angle
        power = joy_drive_data.drive.speed

        angle = self.twist_to_angle(twist)
        speed = self.power_to_speed(power)

        send_signal(angle, speed)
    
    def twist_to_angle(self, twist):
        center = (MAX_ANGLE+MIN_ANGLE)/2
        bank = MAX_ANGLE-center
        angle = center + (bank * (twist/100))
        return angle

    def power_to_speed(self, power):
        return power * MAX_SPEED

def send_signal(angle, speed):
    arduino.flushOutput()
    message = str(angle) + '\n' + str(speed)
    arduino.write(message.encode())

def main():
    rospy.init_node('arduino_controller')
    rate = rospy.Rate(10)
    ArduinoController_obj = ArduinoController()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        send_signal(KILL_ANGLE,KILL_SPEED)
        pass
