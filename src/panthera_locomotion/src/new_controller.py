#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Empty
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse
from ds4_driver.msg import Status as st

class Ds4Controller():
	def __init__(self):
		rospy.init_node('Controller')
		self.mode = 1
		self.brush = Button(0,1)
		self.act = Button(-1,1)
		self.vac = Button(0,1)
		rospy.Subscriber('/cmd_vel', Twist, self.cmd_sub)
		rospy.Subscriber('/status', st, self.ds4_sub)
		rospy.Subscriber('/can_encoder', Twist, self.encoder_pos)

		self.pub = rospy.Publisher('/panthera_cmd', Twist, queue_size=1)
		self.recon = rospy.Publisher('/reconfig', Twist, queue_size=1)

		#### 
		self.brushes = rospy.Publisher('/linear_actuators', Twist, queue_size=1)
		self.actuators = rospy.Publisher('/actuators_topic', Twist, queue_size=1)
		self.vacuum = rospy.Publisher('/vacuum_topic', Twist, queue_size=1)
		###
		
		rospy.wait_for_service('/lb_steer_status')
		rospy.wait_for_service('/lf_steer_status')
		rospy.wait_for_service('/rb_steer_status')
		rospy.wait_for_service('/rf_steer_status')

		self.lb_status = rospy.ServiceProxy('lb_steer_status', Status)
		self.lf_status = rospy.ServiceProxy('lf_steer_status', Status)
		self.rb_status = rospy.ServiceProxy('rb_steer_status', Status)
		self.rf_status = rospy.ServiceProxy('rf_steer_status', Status)
		
		rospy.wait_for_service('/lb_reconfig_status')
		rospy.wait_for_service('/lf_reconfig_status')
		rospy.wait_for_service('/rb_reconfig_status')
		rospy.wait_for_service('/rf_reconfig_status')

		self.lb_stat = rospy.ServiceProxy('lb_reconfig_status', Status)
		self.lf_stat = rospy.ServiceProxy('lf_reconfig_status', Status)
		self.rb_stat = rospy.ServiceProxy('rb_reconfig_status', Status)
		self.rf_stat = rospy.ServiceProxy('rf_reconfig_status', Status)
		
		self.width = 0.6
		self.length = 1.31

		self.linear_x = 0
		self.angular_z = 0

		self.rot_right = 0
		self.rot_left = 0

		self.holo_right = 0
		self.holo_left = 0

		self.rec_r = 0
		self.rec_l = 0

		self.d_vx = 0
		self.d_wz = 0
		self.decrease = 0

		#### Initial VX, WZ and reconfig speed ####
		self.vx = 0.045
		self.wz = 0.04618
		self.reconfig_vel = 0.1

		self.step = 0.01 # Step to adjust speed
		###########################################

		self.twist = Twist()
		self.reconfiguring = Twist()
		self.input_list = []

		self.pub_once = 0

	def custom_twist(self, val):
		ts = Twist()
		ts.angular.x = val
		ts.angular.y = val
		ts.angular.z = val
		return ts

	def encoder_pos(self,data):
		self.width = (data.angular.y + data.angular.z)/2

	def cmd_sub(self, data):
		self.linear_x = data.linear.x
		self.angular_z = data.angular.z

	def ds4_sub(self, data):
		self.rot_right = data.button_dpad_right
		self.rot_left = data.button_dpad_left

		self.holo_right = data.button_r1
		self.holo_left = data.button_l1

		self.rec_r = data.button_r2
		self.rec_l = data.button_l2

		self.d_vx = data.button_triangle# and (not data.button_share)
		self.d_wz = data.button_cross# and (not data.button_share)
		self.decrease = -data.button_share

		###
		self.brush.value = data.button_options
		self.act.value = data.button_square
		self.vac.value = data.button_circle
		self.brush.change_state()
		self.act.change_state()
		self.vac.change_state()
		###

		self.input_list = [self.linear_x, self.angular_z, self.rot_right, self.rot_left,
						   self.holo_right, self.holo_left, self.d_vx, self.d_wz, self.decrease, self.rec_r, self.rec_l]


	def change_vx(self):
		if self.d_vx == 0:
			pass
		else:
			while self.d_vx == 1:
				pass
			self.vx += (1*(not self.decrease) + self.decrease) * self.step

	def change_wz(self):
		if self.d_wz == 0:
			pass
		else:
			while self.d_wz == 1:
				pass
			self.wz += (1*(not self.decrease) + self.decrease) * self.step

	def check(self):
		req = StatusRequest()
		req.reconfig = True
		signal = False
		rate = rospy.Rate(2)
		while signal == False and not rospy.is_shutdown():
			rate.sleep()
			lb = self.lb_status(req)
			rb = self.rb_status(req)
			lf = self.lf_status(req)
			rf = self.rf_status(req)
			signal = (lb.status and rb.status and lf.status and rf.status)
			print([lb.status, rb.status, lf.status, rf.status])
			print("Status of steering motors:" + str(signal))

	def reconfig(self, state):
		req = StatusRequest()
		req.reconfig = state
		stat = not state
		while stat!= state:
			lb = self.lb_stat(req)
			lf = self.lf_stat(req)
			rb = self.rb_stat(req)
			rf = self.rf_stat(req)
			stat = (lb.status and rb.status and lf.status and rf.status)

	def adjust_wheels(self, vx, wz): # radius in m, direction c(-1) or ccw(1)
		if wz == 0:
			radius = float('inf')
		else:
			radius = vx/wz
		left = radius - self.width/2
		right = radius + self.width/2
		
		lf = round(math.degrees(math.atan((self.length*0.5) / left)), 2)
		rf = round(math.degrees(math.atan((self.length*0.5) / right)), 2)
		lb = -lf
		rb = -rf
		return (lb,rb,lf,rf)

	def filter_input(self, x):
		output = 0
		if x < -90:
			output = -90
		elif x > 90:
			output = 90
		elif -90 <= x <= 90:
			output = x
		return output

	def locomotion(self):
		self.mode = 1 * (not self.rot_right) * (not self.rot_left) * (not self.holo_right) * (not self.holo_left) * (not self.rec_l) * (not self.rec_r)
		self.reconfig(not self.mode)
		self.change_wz()
		self.change_vx()
		f = self.linear_x * self.vx
		s = self.angular_z * self.wz

		if f == 0:
			s = (-self.rot_right + self.rot_left) * self.wz
		elif f < 0:
			s = -s
		else:
			pass

		h_r = self.holo_right * 90
		h_l = -self.holo_left * 90
		recon_r = self.rec_r * 90
		recon_l = self.rec_l * 90
		recon_move = (self.rec_r or self.rec_l) * f
		lb,rb,lf,rf = self.adjust_wheels(f, s)
		self.twist.linear.x = self.filter_input(lb - h_r - h_l + recon_l)
		self.twist.linear.y = self.filter_input(rb - h_r - h_l + recon_r)
		self.twist.linear.z = self.filter_input(lf - h_r - h_l + recon_l)
		self.twist.angular.x = self.filter_input(rf - h_r - h_l + recon_r)
		#self.twist.angular.y = 0
		#self.twist.angular.z = 0
		self.pub.publish(self.twist)

		if self.mode != 0:
			pass
		else:
			self.check()
			#pass

		self.reconfiguring.linear.x = self.rec_l * recon_move
		self.reconfiguring.linear.y = self.rec_r * recon_move
		self.reconfiguring.linear.z = self.rec_l * recon_move
		self.reconfiguring.angular.x = self.rec_r * recon_move
		self.recon.publish(self.reconfiguring)

		self.twist.angular.y = f #* (not self.rec_r and not self.rec_l)
		self.twist.angular.z = s #* (not self.rec_r and not self.rec_l)
		self.pub.publish(self.twist)

	def run(self):
		b = self.custom_twist(self.brush.data*100)
		self.brushes.publish(b)
		a = self.custom_twist(self.act.data*100)
		self.actuators.publish(a)
		v = self.custom_twist(self.vac.data*100)
		self.vacuum.publish(v)
		if sum(self.input_list) != self.pub_once:
			if sum(self.input_list) > 5:
				print("Error: Pressing more than 2 buttons")
			else:
				self.pub_once = sum(self.input_list)
				self.locomotion()
				#self.print_instructions()
		else:
			self.pub_once = sum(self.input_list)

		#print(sum(self.input_list), self.mode)

	def print_instructions(self):
		print('\n')
		print("    MOVEMENT    ")
		print("    --------    ")
		#print("[up]: Forward")
		#print("[right]: Rotate Right")
		#print("[left]: Rotate Left")
		#print("[down]: Reverse" + '\n')
		print("[Left joystick]: Forward/Backwards")
		print("[Right joystick]: Left/Right")
		print("[left]: Rotate Left")
		print("[Right]: Rotate Right" + '\n')

		print("    HOLONOMIC/RECONFIGURATION MOVEMENT    ")
		print("    ----------------------------------    ")
		print("[r1] + [up]: Holonomic Right")
		print("[l1] + [up]: Holonomic Left")
		print("[r2] + [up]/[down]: Right Contract/Expand")
		print("[l2] + [down]/[up]: Right Contract/Expand" + '\n')

		print("    ADJUST SPEED    ")
		print("    ------------    ")
		#print("[left] + [right/left]: Turn Right/Left")
		#print("[down] + [right/left]: Reverse Right/Left")
		print("[Triangle/Cross]: + VX/WZ")
		print("[share] + [triangle/cross]: - VX/WZ" + '\n')

		print("    Current Velocity: ")
		print("    -----------------")
		print("VX: " + str(self.vx))
		print("WZ: " + str(self.wz))
		print("Turning Radius: " + str(round(self.vx/self.wz,2)))
		print("Robot Width: " + str(self.width) + '\n')

		print("    Cleaning:")
		print("    ---------")
		print("[options]: Brushes -> " + str(self.brush))
		print("[square]: Actuators -> " + str(self.act))
		print("[circle]: Vacuum -> " + str(self.vac))
		#print("Mode (0->reconfig , 1->smooth): " + str(self.mode))

class Button():
	def __init__(self, low_limit, high_limit):
		self.state = 0
		self.value = 0
		self.check = 0
		self.data = 0
		self.low_limit = low_limit
		self.high_limit = high_limit

	def change_state(self):
		if self.check == self.value:
			pass
		else:
			if self.value == 1 and self.state == 0:
				self.state = 1 # init press
				self.data = self.high_limit

			elif self.value == 0 and self.state == 1:
				self.state = 2 # button pressed
				self.data = self.high_limit

			elif self.value == 1 and self.state == 2:
				self.state = 0
				self.data = self.low_limit

			else:
				pass

		self.check = self.value


if __name__ == "__main__":
	start = Ds4Controller()
	rate = rospy.Rate(10)
	start.print_instructions()
	while not rospy.is_shutdown():
		#start.print_instructions()
		start.run()
		rate.sleep()