#!/usr/bin/env python

'''
Things to note for each motor node:
1. Address #
2. Port /dev/ttyUSB* if using multiple USB ports
3. PID parameters
4. Subscribing to correct twist message for both subscribers
5. Node name
'''

import rospy
import time
import orienbus
import serial.tools.list_ports
import math

from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusResponse
from steer_control import SteerMotor

if __name__ == "__main__":
    try:
        rospy.init_node("lb_steering_motor")
        lb_steer = SteerMotor('lb', 13)
        period = 0.05
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            start = rospy.get_time()
            lb_steer.adjust_speed(period)
            rate.sleep()
            end = rospy.get_time()
            period = end - start
    except rospy.ROSInterruptException:
        pass
