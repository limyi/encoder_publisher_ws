#!/usr/bin/env python

import rospy
import math
import orienbus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt32
import serial.tools.list_ports

class TransMotor():

	def __init__(self, name, address, sign):
		#rospy.init_node('rf_trans_motor')
		rospy.Subscriber('/panthera_cmd', Twist, self.callback)
		rospy.Subscriber('/reconfig', Twist, self.reconfig)
		rospy.Subscriber('/can_encoder', Twist, self.encoder_position) # CHANGE THE TOPIC NAME
		self.wheel_vel_pub = rospy.Publisher('/{}_wheel_vel'.format(name), Float32, queue_size=1)
		self.tolerance = rospy.get_param('/angle_tolerance')

		self.sign = sign
		self.address = address
		self.name = name
		p = list(serial.tools.list_ports.grep(rospy.get_param('/{}_sn'.format(name))))
		self.port = '/dev/' + p[0].name
		self.orienbus = orienbus.OrienBus(self.port)
		self.motor = self.orienbus.initialize(self.address)

		self.width = 0.66
		self.length = 1.34
		self.wheel_radius = 0.1
		self.gear_ratio = 100

		self.linear_x = 0
		self.angular_z = 0
		self.wheel_speed = 0
		self.reconfig_speed = 0

		self.wheel_velocity = 0
		self.motor_rpm = 0

	def encoder_position(self, data):
		if self.name == 'lb':
			self.position = data.linear.x
			self.complement = data.linear.y

		elif self.name == 'rb':
			self.position = data.linear.y
			self.complement = data.linear.x

		elif self.name == 'lf':
			self.position = data.linear.z
			self.complement = data.angular.x

		elif self.name == 'rf':
			self.position = data.angular.x
			self.complement = data.linear.z

		self.width = (data.angular.z + data.angular.y)/2

	def reconfig(self, data): ###
		if self.name == 'lb':
			self.reconfig_speed = data.linear.x
		elif self.name == 'rb':
			self.reconfig_speed = data.linear.y
		elif self.name == 'lf':
			self.reconfig_speed = data.linear.z
		elif self.name == 'rf':
			self.reconfig_speed = data.angular.x
		self.wheel_speed = self.rads_to_rpm(self.reconfig_speed / self.wheel_radius)

	def callback(self, data):
		self.linear_x = data.angular.y
		self.angular_z = data.angular.z

	def motor_lin_vel(self, vx, wz):
		sign = wz / abs(wz)
		r = vx / wz
		rot_dir = vx / abs(vx)
		speed = rot_dir*(math.sqrt((r - self.sign*self.width/2)**2 + (self.length/2)**2) * abs(wz)) # check + or - 
		if abs(r) < self.width/2:
			speed  = -sign*self.sign*speed*rot_dir
		else:
			pass
		return speed # check motor direction 

	def rads_to_rpm(self, x):
		rpm = (x / (2*math.pi)) * 60 * self.gear_ratio
		return int(-rpm)

	def rpm_to_rads(self, x):
		rads = (x/60) * 2 * math.pi / self.gear_ratio
		return (-rads)

	def adjust_speed(self, vx, wz):
		speed = 0
		#if run_mode == True:
		if vx == 0:
			if wz == 0:
				rpm = 0
				speed = 0
				self.motor_rpm = rpm
				self.motor.writeSpeed(rpm)
				#print(" rpm: 0")

			else:
				speed = -self.sign*wz * math.sqrt((self.length/2)**2 + (self.width/2)**2) / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor_rpm = rpm
				self.motor.writeSpeed(rpm)			

		else:
			if wz == 0:
				speed = vx / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor_rpm = rpm
				self.motor.writeSpeed(rpm)

			else:
				
				lin_vel  = self.motor_lin_vel(vx, wz)
				speed = lin_vel / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
				#print("lf rpm: " + str(rpm))
				self.motor_rpm = rpm
				self.motor.writeSpeed(rpm)
				
				#self.control_speed(vx, wz)
	'''
	def control_speed(self, vx, wz):
		if self.name == "rf" or self.name == "rb":
			if self.position <= 0:
				lin_vel  = self.motor_lin_vel(vx, wz)
				speed = lin_vel / self.wheel_radius
				rpm = self.rads_to_rpm(speed)

			else:
				lin_vel  = self.motor_lin_vel(vx, wz)
				if self.complement == 0:
					speed = lin_vel / self.wheel_radius
				else:
					speed = lin_vel / self.wheel_radius * abs(math.sin(self.position)/math.sin(self.complement))
				rpm = self.rads_to_rpm(speed)

		elif self.name == "lf" or self.name == "lb":
			if self.position >= 0:
				lin_vel  = self.motor_lin_vel(vx, wz)
				speed = lin_vel / self.wheel_radius
				rpm = self.rads_to_rpm(speed)

			else:
				lin_vel  = self.motor_lin_vel(vx, wz)
				if self.complement == 0:
					speed = lin_vel / self.wheel_radius
				else:
					speed = lin_vel / self.wheel_radius * abs(math.sin(self.position)/math.sin(self.complement))
				rpm = self.rads_to_rpm(speed)
		self.motor.writeSpeed(rpm)
	'''	
	def control_speed(self, vx, wz):
		direction = (vx/wz) / abs(vx/wz)
		if direction < 0:
			if self.name == "lb" or self.name == "lf":
				lin_vel  = self.motor_lin_vel(vx, wz)
				speed = lin_vel / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
			
			elif self.name == "rb" or self.name == "rf":
				if abs(self.position) <= self.tolerance:
					lin_vel = wz * self.length / (2 * abs(math.sin(self.complement)))
					speed = lin_vel / self.wheel_radius
					rpm = -self.rads_to_rpm(speed)

				else:
					lin_vel = wz * self.length / (2 * abs(math.sin(self.position)))
					speed = lin_vel / self.wheel_radius
					rpm = -self.rads_to_rpm(speed)

			self.motor_rpm = rpm
			self.motor.writeSpeed(rpm)	

		elif direction > 0:
			if self.name == "rb" or self.name == "rf":
				lin_vel  = self.motor_lin_vel(vx, wz)
				speed = lin_vel / self.wheel_radius
				rpm = self.rads_to_rpm(speed)
			
			elif self.name == "lb" or self.name == "lf":
				if abs(self.position) <= self.tolerance:
					lin_vel = wz * self.length / (2 * abs(math.sin(self.complement)))
					speed = lin_vel / self.wheel_radius
					rpm = self.rads_to_rpm(speed)

				else:
					lin_vel = wz * self.length / (2 * abs(math.sin(self.position)))
					speed = lin_vel / self.wheel_radius
					rpm = self.rads_to_rpm(speed)
			self.motor_rpm = rpm
			self.motor.writeSpeed(rpm)
	
	def pub_wheel_vel(self):
		self.wheel_velocity = self.sign*self.motor.readSpeed()#self.sign * self.rpm_to_rads(self.motor.readSpeed()) * self.wheel_radius
		self.wheel_vel_pub.publish(self.wheel_velocity)
		#print(self.name, self.wheel_velocity, self.motor_rpm)