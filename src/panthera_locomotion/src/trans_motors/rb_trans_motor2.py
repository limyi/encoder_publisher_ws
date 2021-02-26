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
		rospy.init_node('rb_trans_motor')
		rb_motor = TransMotor('rb', 3, -1)
		#rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			if rb_motor.wheel_speed == 0:
				rb_motor.adjust_speed(rb_motor.linear_x, rb_motor.angular_z)
				#rb_motor.control_speed(rb_motor.linear_x, rb_motor.angular_z)
			else:
				#print("lf rpm: " + str(lf_motor.wheel_speed))
				rb_motor.motor.writeSpeed(rb_motor.wheel_speed)
			rb_motor.pub_wheel_vel()
			#rate.sleep()
	except rospy.ROSInterruptException:
		pass