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
		self.name = name # lb, rb, lf or rf
		self.address = address # motor address
		# Specific motor parameters
		rospy.Subscriber('/can_encoder', Twist, self.encoder_pos) # subscriber to steering and wire encoders
		rospy.Subscriber('/panthera_cmd', Twist, self.desired_pos) # subscibe to desired angles and cmd_vel for robot
		service = rospy.Service('{}_reconfig_status'.format(name), Status, self.callback) # service to set status for steering independently or ackermann
		serv = rospy.Service('{}_steer_status'.format(name), Status, self.steer_stat) # service to check if wheels have reached desired angle

		# PID params
		self.kp = rospy.get_param('{}_pid'.format(self.name))['kp']
		self.ki = rospy.get_param('{}_pid'.format(self.name))['ki']
		self.kd = rospy.get_param('{}_pid'.format(self.name))['kd']

		# Max possible speed for steering motors
		self.MAX_SPEED = rospy.get_param('/{}_max_speed'.format(self.name))
		self.sn = rospy.get_param('/{}_serial_number'.format(self.name)) # Serial number for RS485 cable to motor
		self.tolerance = rospy.get_param('/angle_tolerance') # Stop motor when wheel reaches within this tolerance from desired angle

		# set higher tolerance for lb motor due to different gear 
		if self.name == "lb":
			self.tolerance+=0.5
		self.integral_reset = rospy.get_param('/{}_pid'.format(self.name))['ir'] # intergral term resets when reaching this value

		# code to attach serial port to motor
		p = list(serial.tools.list_ports.grep(self.sn))
		self.port = '/dev/' + p[0].name
		self.orienbus = orienbus.OrienBus(self.port)
		self.motor = self.orienbus.initialize(self.address)

		# Angles and erros
		self.target = 0 # desired angle for wheel
		self.target_change = 0 # diff between current angle and desired angle
		self.position = 0 # current angle of wheel
		self.complement = 0 # angle of complement wheel. Complements: (lb and rb), (lf and rf)
		self.complement_target = 0 # target angle for complement wheel
		self.complement_error = 0 # diff between target and current angle for complement wheel

		# errors for pid controller
		self.current_error = 0 
		self.prev_error = 0
		self.accu_error = 0

		# current steering motor speed
		self.motor_speed = 0

		self.control_activation = 3 # angle when pid control is activatied
		self.inner_max_speed = 800 # fixed speed for inner wheel when turning

		self.reconfig = True # mode if wheels can rotate independent of complement

	def callback(self, req):
		self.reconfig = req.reconfig # get mode for steering
		return StatusResponse(status=self.reconfig, speed=self.motor_speed)

	def steer_stat(self, req):
		# when in reconfig mode ie: static rotation, holonomic movement and reconfiguration
		if req.reconfig == True:
			# return true if wheels have reached desired angle, return false if not reached 
			if abs(self.current_error) <= self.tolerance:
				return StatusResponse(status=True, speed=self.motor_speed)
			else:
				return StatusResponse(status=False, speed=self.motor_speed)
		# do nothing if not in reconfig mode
		else:
			pass

	def encoder_pos(self, data):
		# order for wheel angles in twist msg: lb,rb,lf,rf
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

	def desired_pos(self, data):
		# reading desired angle for each wheel
		if self.name == 'lb':
			self.target = data.linear.x
			self.complement_target = data.linear.y

		elif self.name == 'rb':
			self.target = data.linear.y
			self.complement_target = data.linear.x

		elif self.name == 'lf':
			self.target = data.linear.z
			self.complement_target = data.angular.x

		elif self.name == 'rf':
			self.target = data.angular.x
			self.complement_target = data.linear.z

	#### PID controller ######
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
	###########################

	def adjust_max_speed(self):
		# if complement wheel has reached target angle, stop current wheel to avoid oscillation
		if abs(self.complement_error) == 0:
			self.MAX_SPEED = 0
		else:
			if self.name == "lf":
				if self.position >= 0: # if wheel is facing to the left
					self.MAX_SPEED = self.inner_max_speed # lf is inner wheel
				elif abs(self.position) < self.tolerance and self.position < 0:
					self.MAX_SPEED = self.inner_max_speed # if wheel close to 0 but facing right, consider it as inner wheel
				else:
					self.MAX_SPEED = abs(self.current_error) * self.inner_max_speed / abs(self.complement_error) # lf is outer wheel

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
		# get values from PID calculation
		self.current_error = self.target - self.position
		self.complement_error = self.complement_target - self.complement
		self.accu_error += self.current_error
		p = self.proportional(self.target, self.position)
		d = self.derivative(self.current_error, self.prev_error, dt)
		i = self.integral(self.accu_error, dt)
		speed = p + i + d
		speed =  -int(speed)

		# Steering mode
		if self.reconfig == False: # ackerman steering
			self.adjust_max_speed() # set correct speed for wheel
			if abs(self.current_error) >= self.tolerance: # if wheel has not reached target angle and outside tolerance
				if abs(self.current_error) <= self.control_activation: # activated pid control
					self.motor_speed = speed
					self.motor.writeSpeed(speed)
				else:
					# run wheel at max speed when outside control activation for smoother steering
					# control direction of steering
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
				# stop motor if within tolerance
				self.motor_speed = 0
				self.motor.writeSpeed(0)

		# Reconfiguration mode
		else:
			# lb has different gear ratio
			# wheel rotation speed during reconfig mode
			if self.name == 'lb':
				self.MAX_SPEED = 300
			else:
				self.MAX_SPEED = 600
			if abs(self.current_error) >= self.tolerance: # while not reached desired angle
				# while not running at max speed
				if abs(speed) < abs(self.MAX_SPEED):
					self.motor_speed = speed
					self.motor.writeSpeed(speed)
					#print("Speed: "+str(speed))
				# control direction of motor
				else:
					if speed < 0:
						self.motor_speed = -self.MAX_SPEED
						self.motor.writeSpeed(-self.MAX_SPEED)
					else:
						self.motor_speed = self.MAX_SPEED
						self.motor.writeSpeed(self.MAX_SPEED)
			else:
				# stop when reached desired angle
				self.motor_speed = 0
				self.motor.writeSpeed(0)

		self.prev_error = self.current_error # calculating error for PID

		# reset integral term
		if abs(i) >= self.integral_reset:
			self.accu_error = 0
