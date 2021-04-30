#!/usr/bin/env python

import rospy
import time
import orienbus
import serial.tools.list_ports
import math
from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusResponse

class SteerMotor():
	def __init__(self, name, address):
		# Specific motor parameters
		rospy.Subscriber('/encoder_positions', Twist, self.encoder_pos)
		rospy.Subscriber('target_angle', Twist, self.desired_pos)
		service = rospy.Service('{}_reconfig_status'.format(name), Status, self.callback)
		self.name = name
		self.kp = rospy.get_param('{}_pid'.format(self.name))['kp']
		self.ki = rospy.get_param('{}_pid'.format(self.name))['ki']
		self.kd = rospy.get_param('{}_pid'.format(self.name))['kd']
		self.MAX_SPEED = rospy.get_param('/{}_max_speed'.format(self.name))
		self.sn = rospy.get_param('/{}_serial_number'.format(self.name))
		self.tolerance = rospy.get_param('/angle_tolerance')
		self.integral_reset = rospy.get_param('/{}_pid'.format(self.name))['ir']
		self.address = address
		p = list(serial.tools.list_ports.grep(self.sn))
		self.port = '/dev/' + p[0].name
		self.orienbus = orienbus.OrienBus(self.port)
		self.motor = self.orienbus.initialize(self.address)

		# Angles and erros
		self.target = 0
		self.position = 0

		self.current_error = 0
		self.prev_error = 0
		self.accu_error = 0

		self.motor_speed = 0

	def callback(req):
		x = False
		if abs(self.current_error) <= self.tolerance and abs(self.motor_speed) < 80:
			x = True
		else:
			x = False
		#print(x)
		return StatusResponse(status=x,speed=self.motor_speed)

	def encoder_pos(self, data):
		if self.name == 'lb':
			self.position = data.linear.x
		elif self.name == 'rb':
			self.position = data.linear.y
		elif self.name == 'lf':
			self.position = data.linear.z
		elif self.name == 'rf':
			self.position = data.angular.x

	def desired_pos(self, data):
		if self.name == 'lb':
			self.target = data.linear.x
		elif self.name == 'rb':
			self.target = data.linear.y
		elif self.name == 'lf':
			self.target = data.linear.z
		elif self.name == 'rf':
			self.target = data.angular.x

	def proportional(self, desired, actual): #error - current angle
		prop =  self.kp * (desired - actual)
		return prop

	def derivative(self, curr, prev, dt): #d angle-error/ dt
		#dt = 0.05 #current_time - prev_time
		#print("dt is: " + str(dt))
		if dt == 0:
			deriv = 0
		else:
			deriv = self.kd * (curr - prev) / dt
		#print("Kd: "+ str(deriv))
		return deriv

	def integral(self, accu, dt):
		integral =  self.ki * accu * dt
		#print("Ki: " +str(integral))
		return integral

	def control_pid(self, error, s):
		ks = 1.1
		ka = 0.5
		output = ks * s * math.tanh(ka * abs(error) + 1)
		return output

	def adjust_speed(self, dt):
		self.current_error = self.target - self.position
		#print("current error is:" + str(current_error))
		self.accu_error += self.current_error
		p = self.proportional(self.target, self.position)
		d = self.derivative(self.current_error, self.prev_error, dt)
		i = self.integral(self.accu_error, dt)
		'''
		if i > integral_reset:
			i = integral_reset
		if i < -integral_reset:
			i = -integral_reset
		'''
		speed = p + i + d
		if abs(self.current_error) < 0:
			speed = self.control_pid(self.current_error, speed) ###### Extra control
		speed =  -int(speed)
		#print("input speed is: " + str(speed))
		if abs(self.current_error) >= self.tolerance:
			if abs(speed) < abs(self.MAX_SPEED):
				self.motor_speed = speed
				self.motor.writeSpeed(speed)
				#print("Speed: "+str(speed))
			else:
				if speed < 0:
					self.motor_speed = -self.MAX_SPEED
					self.motor.writeSpeed(-self.MAX_SPEED)
					#print("Speed: "+str(-MAX_SPEED))
				else:
					self.motor_speed = self.MAX_SPEED
					self.motor.writeSpeed(self.MAX_SPEED)
					#print("Speed: "+str(MAX_SPEED))
		else:
			self.motor_speed = 0
			self.motor.writeSpeed(0)
		self.prev_error = self.current_error
		#prev_time = current_time
		#print("accu_error: " + str(accu_error))
		
		if abs(i) >= self.integral_reset:
			self.accu_error = 0