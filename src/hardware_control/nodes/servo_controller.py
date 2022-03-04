#!/usr/bin/env python3.7
from adafruit_servokit import ServoKit
import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")
myKit=ServoKit(channels=16)

KILL_ANGLE = 95
KILL_SPEED = 90

ANGLE_PIN = 0
SPEED_PIN = 1

angle = KILL_ANGLE
speed = KILL_SPEED
myKit.servo[0].angle=angle
myKit.servo[1].angle=speed
print("Setup Complete")

while True:
    message = socket.recv()
    message = message.decode('ascii')
    #print(f"Received request: {message}")
    socket.send(str(1).encode())

    pmessage = message.partition("|")
    angle = float(pmessage[0])
    speed = float(pmessage[2])
    print("-------------------")
    print("Angle:" + str(angle))
    print("Speed: " + str(speed))
    myKit.servo[ANGLE_PIN].angle=angle
    myKit.servo[SPEED_PIN].angle=speed