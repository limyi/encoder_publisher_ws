#!/usr/bin/env python

import rospy
import math
import orienbus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial.tools.list_ports
from trans_class import TransMotor

if __name__ == "__main__":
	try:
		rospy.init_node('lf_trans_motor')
		lf_motor = TransMotor('lf', 2, 1)
		while not rospy.is_shutdown():
			if lf_motor.wheel_speed == 0: # normal locomotion
				lf_motor.adjust_speed(lf_motor.linear_x, lf_motor.angular_z)
			else: # reconfig mode
				lf_motor.motor.writeSpeed(lf_motor.wheel_speed)
			lf_motor.pub_wheel_vel()
	except rospy.ROSInterruptException:
		pass





