#!/usr/bin/env python

import rospy
import time
import orienbus
import serial.tools.list_ports
import math
import numpy as np
from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusResponse

class SteerMotor():
	def __init__(self, name, address):
		self.name = name
		self.address = address
		self.min_turning_radius = 1.34
		self.length = 1.34
		self.width = 0.68
		# Specific motor parameters
		rospy.Subscriber('/can_encoder', Twist, self.encoder_pos)
		rospy.Subscriber('/panthera_cmd', Twist, self.cmd_vel)
		service = rospy.Service('{}_reconfig_status'.format(name), Status, self.callback)
		serv = rospy.Service('{}_steer_status'.format(name), Status, self.steer_stat)

		self.kp = rospy.get_param('{}_pid'.format(self.name))['kp']
		self.ki = rospy.get_param('{}_pid'.format(self.name))['ki']
		self.kd = rospy.get_param('{}_pid'.format(self.name))['kd']
		self.MAX_SPEED = rospy.get_param('/{}_max_speed'.format(self.name))
		self.sn = rospy.get_param('/{}_serial_number'.format(self.name))
		self.tolerance = rospy.get_param('/angle_tolerance')
		self.integral_reset = rospy.get_param('/{}_pid'.format(self.name))['ir']

		p = list(serial.tools.list_ports.grep(self.sn))
		self.port = '/dev/' + p[0].name
		self.orienbus = orienbus.OrienBus(self.port)
		self.motor = self.orienbus.initialize(self.address)

		#self.length = 1.34
		#self.width = 0.68

		# Angles and erros
		self.target = 0
		self.position = 0
		self.complement = 0
		self.complement_target = 0
		self.complement_error = 0

		self.current_error = 0
		self.prev_error = 0
		self.accu_error = 0

		self.motor_speed = 0

		self.control_activation = 3
		self.inner_max_speed = 400
		#self.min_turning_radius = 1.34

		self.reconfig = False

	def cmd_vel(self, data): # radius in m, direction c(-1) or ccw(1)
		lf = 0
		rf = 0
		if data.angular.z == 0:
			r = float('inf')
		else:
			r = data.linear.x / data.angular.z
		if abs(r) <= self.min_turning_radius:
			pass
		else:
			if r < 0:
				rf = -math.atan(self.length / (abs(r)-self.width/2))
				lf = -math.atan(self.length / (abs(r)+self.width/2))
			elif r > 0:
				rf = math.atan(self.length / (abs(r)+self.width/2))
				lf = math.atan(self.length / (abs(r)-self.width/2))
			elif r == 0:
				rf = 0
				lf = 0

		if self.name == "lf":
			self.target = lf*360/(2*math.pi)
		elif self.name == "rf":
			self.target = rf*360/(2*math.pi)
		else:
			self.target = 0

	def callback(self, req):
		self.reconfig = req.reconfig
		return StatusResponse(status=self.reconfig, speed=self.motor_speed)

	def steer_stat(self, req):
		if req.reconfig == True:
			if abs(self.current_error) <= self.tolerance:
				return StatusResponse(status=True, speed=self.motor_speed)
			else:
				return StatusResponse(status=False, speed=self.motor_speed)
		else:
			pass

	def encoder_pos(self, data):
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

	def adjust_max_speed(self):
		if abs(self.complement_error) == 0:
			self.MAX_SPEED = 0
		else:
			if self.name == "lf":
				if self.position >= 0:
					self.MAX_SPEED = self.inner_max_speed
				elif abs(self.position) < self.tolerance and self.position < 0:
					self.MAX_SPEED = self.inner_max_speed
				else:
					self.MAX_SPEED = abs(self.current_error) * self.inner_max_speed / abs(self.complement_error)

			elif self.name == "rf":
				if self.position <= 0:
					self.MAX_SPEED = self.inner_max_speed
				elif abs(self.position) < self.tolerance and self.position > 0:
					self.MAX_SPEED = self.inner_max_speed
				else:
					self.MAX_SPEED = abs(self.current_error) * self.inner_max_speed / abs(self.complement_error)

			elif self.name == "lb":
				if self.position <= 0:
					self.MAX_SPEED = self.inner_max_speed/2
				elif abs(self.position) < self.tolerance and self.position > 0:
					self.MAX_SPEED = self.inner_max_speed/2
				else:
					self.MAX_SPEED = abs(self.current_error) * (self.inner_max_speed/2) / abs(self.complement_error)

			elif self.name == "rb":
				if self.position >= 0:
					self.MAX_SPEED = self.inner_max_speed
				elif abs(self.position) < self.tolerance and self.position < 0:
					self.MAX_SPEED = self.inner_max_speed
				else:
					self.MAX_SPEED = abs(self.current_error) * self.inner_max_speed / abs(self.complement_error)
			

	def adjust_speed(self, dt):
		self.current_error = self.target - self.position
		self.complement_error = self.complement_target - self.complement
		self.accu_error += self.current_error
		p = self.proportional(self.target, self.position)
		d = self.derivative(self.current_error, self.prev_error, dt)
		i = self.integral(self.accu_error, dt)
		speed = p + i + d
		speed =  -int(speed)

		# Steering mode
		if self.reconfig == False:
			self.adjust_max_speed()
			if abs(self.current_error) >= self.tolerance:
				if abs(self.current_error) <= self.control_activation:
					self.motor_speed = speed
					self.motor.writeSpeed(speed)
				else:
					if speed < 0:
						self.motor_speed = -self.MAX_SPEED
						self.motor.writeSpeed(-self.MAX_SPEED)

					elif speed == 0:
						self.motor_speed = 0
						self.motor.writeSpeed(0)

					else:
						self.motor_speed = self.MAX_SPEED
						self.motor.writeSpeed(self.MAX_SPEED)
			else:
				self.motor_speed = 0
				self.motor.writeSpeed(0)

		# Reconfiguration mode
		else:
			if self.name == 'lb':
				self.MAX_SPEED = 300
			else:
				self.MAX_SPEED = 600
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

		if abs(i) >= self.integral_reset:
			self.accu_error = 0
		'''
		if self.name == 'lf' or self.name == 'rf':
			print(self.name + ': ' + str(self.current_error))
	'''