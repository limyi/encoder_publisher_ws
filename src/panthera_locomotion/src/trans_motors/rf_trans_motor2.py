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
		rospy.init_node('rf_trans_motor')
		rf_motor = TransMotor('rf', 1, -1)
		while not rospy.is_shutdown():
			if rf_motor.wheel_speed == 0: # normal locomotion
				rf_motor.adjust_speed(rf_motor.linear_x, rf_motor.angular_z)
			else: # reconfig mode
				rf_motor.motor.writeSpeed(rf_motor.wheel_speed)
			rf_motor.pub_wheel_vel()
	except rospy.ROSInterruptException:
		pass

