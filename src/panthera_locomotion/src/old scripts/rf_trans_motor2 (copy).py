#!/usr/bin/env python

import rospy
import math
import orienbus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial.tools.list_ports

class TransMotor():

	def __init__(self):
		#rospy.init_node('rf_trans_motor')
		rospy.Subscriber('/target_angle', Twist, self.callback)
		rospy.Subscriber('/reconfig', Twist, self.reconfig)
		self.wheel_vel_pub = rospy.Publisher('/rf_wheel_vel', Float32, queue_size=1)

		self.address = 1
		p = list(serial.tools.list_ports.grep(rospy.get_param('/rf_sn')))
		self.port = '/dev/' + p[0].name
		self.orienbus = orienbus.OrienBus(self.port)
		self.motor = self.orienbus.initialize(self.address)

		self.width = 0.89
		self.length = 1.31
		self.wheel_radius = 0.1
		self.gear_ratio = 100

		self.linear_x = 0
		self.angular_z = 0
		self.wheel_speed = 0
		self.reconfig_speed = 0

		self.wheel_velocity = 0
		#self.motor_rpm = 0

	def reconfig(self, data): ###
		self.reconfig_speed = data.angular.x
		self.wheel_speed = -self.rads_to_rpm(data.angular.x / self.wheel_radius)

	def callback(self, data):
		self.linear_x = data.angular.y
		self.angular_z = data.angular.z

	def motor_lin_vel(self, vx, wz):
		sign = wz / abs(wz)
		r = abs(vx / wz)
		speed = math.sqrt((r - sign*self.width/2)**2 + (self.length/2)**2) * abs(wz)
		return speed # check motor direction 

	def rads_to_rpm(self, x):
		rpm = (x / (2*math.pi)) * 60 * self.gear_ratio
		return int(-rpm)

	def adjust_speed(self, vx, wz):
		speed = 0
		#if run_mode == True:
		if vx == 0:
			if wz == 0:
				rpm = 0
				speed = 0
				self.motor.writeSpeed(rpm)
				#print("lf rpm: 0")

			else:
				speed = -wz * math.sqrt((self.length/2)**2 + (self.width/2)**2) / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor.writeSpeed(rpm)			

		else:
			if wz == 0:
				speed = vx / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor.writeSpeed(rpm)

			else:
				lin_vel  = self.motor_lin_vel(vx, wz)
				speed = lin_vel / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor.writeSpeed(rpm)

	def pub_wheel_vel(self):
		self.wheel_velocity = -self.motor.readSpeed() * self.gear_ratio * self.wheel_radius
		self.wheel_vel_pub.publish(self.wheel_velocity)

if __name__ == "__main__":
	try:
		rospy.init_node('rf_trans_motor')
		rf_motor = TransMotor()
		#rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			if rf_motor.wheel_speed == 0:
				rf_motor.adjust_speed(rf_motor.linear_x, rf_motor.angular_z)

			else:
				#print("lf rpm: " + str(lf_motor.wheel_speed))
				rf_motor.motor.writeSpeed(rf_motor.wheel_speed)
			rf_motor.pub_wheel_vel()
			#rate.sleep()
	except rospy.ROSInterruptException:
		pass

