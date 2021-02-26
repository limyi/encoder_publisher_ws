#!/usr/bin/env python

import rospy
import time
import orienbus
import serial.tools.list_ports
import math

from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusResponse
from steer_class import SteerMotor

if __name__ == "__main__":
    try:
        rospy.init_node("rf_steering_motor") # Note correct node name
        rf_steer = SteerMotor('rf', 10)
        period = 0.05
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if abs(rf_steer.position) >= 95:
                rf_steer.motor.writeSpeed(0)
                break
            else:
                start = rospy.get_time()
                rf_steer.adjust_speed(period)
                rate.sleep()
                end = rospy.get_time()
                period = end - start
    except rospy.ROSInterruptException:
        pass
