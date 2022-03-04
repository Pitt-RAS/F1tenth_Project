#!/usr/bin/env python2

import zmq
import time


import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
import math

import numpy as np


bitrate = 115200
port = '/dev/ttyACM0'

MAX_ANGLE = 130
MIN_ANGLE = 60

MAX_SPEED = 15

KILL_ANGLE = 95
KILL_SPEED = 90





context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")



class ServoController:
    def __init__(self):

        self.joy_drive_sub = rospy.Subscriber('joy_drive', AckermannDriveStamped, self.joy_drive_callback)

    def joy_drive_callback(self, joy_drive_data):
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
        return ((power / 100) * MAX_SPEED) + 90


def send_signal(angle, speed):
    #print "Signal Sent"
    message = str(angle) + "|" + str(speed)
    socket.send(message.encode())
    message = socket.recv()


def main():
    rospy.init_node('servo_controller')
    rate = rospy.Rate(10)
    ServoController_obj = ServoController()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        send_signal(KILL_ANGLE,KILL_SPEED)
        pass