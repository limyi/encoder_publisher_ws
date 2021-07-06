#!/usr/bin/env python3

#### Created by SUTD ROAR LAB ####

import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import serial
import struct
import time
import sys
import linbus_ultrasonic as ultra
from bitarray import bitarray # pip install bitarray
import bitarray.util as util
from binascii import hexlify
import serial.tools.list_ports

class RosHandler:

    def __init__(self, port):
        self.data_publisher = rospy.Publisher("/ultrasonic_data", Twist, queue_size=3)
        print("Running")
        self.sensors = ultra.Ultrasonic(port) # Check serial port address

    def run(self):
        #addr = input("Enter address: ")
        #readings = self.sensors.measure(addr)
        readings = self.sensors.measure(1)
        print(readings)
        readings = self.sensors.measure(2)
        print(readings)

    def set_address(self):
        curr_addr = input("Enter current address: ")
        new_addr = input("Enter new address: ")
        sensors.setAddress(curr_addr, new_addr)

    def offPWM(self):
        addr = str(input("Enter address: "))
        sensors.off_pwm(addr)

if __name__ == '__main__':
    rospy.init_node('ultrasonic_sensor')
    rate = rospy.Rate(10) # 10hz
    fr = RosHandler("/dev/ttyUSB0")
    # = RosHandler("/dev/ttyUSB1")
    while not rospy.is_shutdown():
        usr_inp = input("Enter mode:\n 1. Set Address\n 2. Measure\n 3. Off PWM\n")
        if usr_inp == "1":
            fr.set_address()
            #s.set_address()
        elif usr_inp == "2":
            fr.run()
            #s.run()
        elif usr_inp == 4:
            fr.measure_all()
            #s.run()
        elif usr_inp == 3:
            fr.offPWM()
            #s.offPWM()
