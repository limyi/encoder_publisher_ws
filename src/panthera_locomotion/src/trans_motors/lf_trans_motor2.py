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
		#rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			if lf_motor.wheel_speed == 0:
				lf_motor.adjust_speed(lf_motor.linear_x, lf_motor.angular_z)
				#lf_motor.control_speed(lf_motor.linear_x, lf_motor.angular_z)
			else:
				#print("lf rpm: " + str(lf_motor.wheel_speed))
				lf_motor.motor.writeSpeed(lf_motor.wheel_speed)
			lf_motor.pub_wheel_vel()
			#print("lf speed: " + str(lf_motor.wheel_velocity))
			#rate.sleep()
	except rospy.ROSInterruptException:
		pass





