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
		#rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			if rf_motor.wheel_speed == 0:
				rf_motor.adjust_speed(rf_motor.linear_x, rf_motor.angular_z)
				#rf_motor.control_speed(rf_motor.linear_x, rf_motor.angular_z)
			else:
				#print("lf rpm: " + str(lf_motor.wheel_speed))
				rf_motor.motor.writeSpeed(rf_motor.wheel_speed)
			rf_motor.pub_wheel_vel()
			#print("rf speed: " + str(rf_motor.wheel_velocity))
			#rate.sleep()
	except rospy.ROSInterruptException:
		pass

