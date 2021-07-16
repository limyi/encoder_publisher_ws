#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Empty
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse
from ds4_driver.msg import Status as st
from ds4_driver.msg import Feedback
from zed_interfaces.msg import ObjectsStamped

class Ds4Controller():
	def __init__(self):
		rospy.init_node('Controller')

		self.mode = 1 # mode 1:=smooth , mode 0:=reconfig

		# Toggle buttons for roboclaw
		self.brush = Button(0,0.01)
		self.act = Button(-1,1)
		self.vac = Button(-1,1)
		self.vision = Button(0,1)

		rospy.Subscriber('/cmd_vel', Twist, self.cmd_sub)
		rospy.Subscriber('/status', st, self.ds4_sub)
		rospy.Subscriber('/can_encoder', Twist, self.encoder_pos)

		### VISION ###
		rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, self.human_loc)
		self.human_dist = float('inf')
		self.human_stop = 1.5
		##############

		self.pub = rospy.Publisher('/panthera_cmd', Twist, queue_size=1)
		self.recon = rospy.Publisher('/reconfig', Twist, queue_size=1)
		self.vibrate = rospy.Publisher('/set_feedback', Feedback, queue_size=1)

		####  publisher for roboclaw
		self.brushes = rospy.Publisher('/linear_actuator', Twist, queue_size=1)
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

		# cmd vel
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
		self.vx = 0.12
		self.wz = 0.08618
		self.reconfig_vel = 0.1

		self.step = 0.01 # Step to adjust speed
		###########################################

		self.twist = Twist()
		self.reconfiguring = Twist()
		self.input_list = []

		self.pub_once = 0

		self.vb = Feedback()
		self.vb.set_rumble = True
		self.vb.rumble_duration = 0.5
		self.vb.rumble_small = 0.5

	def human_loc(self, data):
		if self.vision.data == 1:
			nearest_human = float('inf')
			for person in data.objects:
				if person.position[0] <= nearest_human:
					nearest_human = person.position[0]
			self.human_dist = nearest_human
		else:
			self.human_dist = float('inf')
		#print(self.human_dist)

	def custom_twist(self, val1, val2):
		ts = Twist()
		ts.linear.x = val1
		ts.linear.y = val1
		ts.linear.z = val1
		ts.angular.x = val2
		ts.angular.y = val2
		ts.angular.z = val2
		return ts

	def encoder_pos(self,data):
		lb = data.linear.x
		rb = data.linear.y
		lf = data.linear.z
		rf = data.angular.x
		wheels = [lb,rb,lf,rf]
		for i in wheels:
			if abs(i) > 100:
				#print(i)
				self.vibrate.publish(self.vb)
		self.width = (data.angular.y + data.angular.z)/2
		#self.width = data.angular.z

	def cmd_sub(self, data):
		self.linear_x = data.linear.x
		self.angular_z = data.angular.z

	def ds4_sub(self, data):
		self.rot_right = data.button_dpad_right
		self.rot_left = data.button_dpad_left

		## VISION ##
		self.vision.value = data.button_dpad_up
		self.vision.change_state()
		############
		self.holo_right = data.button_r1
		self.holo_left = data.button_l1

		self.rec_r = data.button_r2
		self.rec_l = data.button_l2

		self.d_vx = data.button_triangle# and (not data.button_share)
		self.d_wz = data.button_cross# and (not data.button_share)
		self.decrease = -data.button_share

		### Brushes roboclaw stuff
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
		prev_mode = self.mode

		# determine mode: mode 1 -> smooth | mode 0 -> reconfig
		self.mode = 1 * (not self.rot_right) * (not self.rot_left) * (not self.holo_right) * (not self.holo_left) * (not self.rec_l) * (not self.rec_r)
		self.reconfig(not self.mode)

		if sum(self.input_list) == 0:
			self.reconfig(True)

		# update vx and wz
		f = self.linear_x * self.vx
		self.change_vx()
		self.change_wz()
		s = self.angular_z * self.wz

		### control max wz wrt to vx ###
		if s >= 0:
			s = min(abs(s), abs(f/(self.width+0.2/2)))
		else:
			s = max(s, -abs(f/(self.width+0.2/2)))
		
		### joystick calibration for reverse turning ###
		if f == 0:
			s = (-self.rot_right + self.rot_left) * self.wz
		elif f < 0:
			s = -s
		else:
			pass

		# determine and publish wheel angles and speed
		h_r = self.holo_right * 90
		h_l = -self.holo_left * 90
		recon_r = -self.rec_r * 90
		recon_l = self.rec_l * 90
		recon_move = (self.rec_r or self.rec_l) * f
		lb,rb,lf,rf = self.adjust_wheels(f, s)
		self.twist.linear.x = self.filter_input(lb - h_r - h_l + recon_l)
		self.twist.linear.y = self.filter_input(rb - h_r - h_l + recon_r)
		self.twist.linear.z = self.filter_input(lf - h_r - h_l + recon_l)
		self.twist.angular.x = self.filter_input(rf - h_r - h_l + recon_r)
		self.pub.publish(self.twist)

		# check wheels algined
		if self.mode != 0:
			if prev_mode == 0:
				self.twist.angular.y = 0
				self.twist.angular.z = 0
				self.pub.publish(self.twist)
				self.check()
			elif self.rec_l == 1 or self.rec_r == 1 or self.holo_left == 1 or self.holo_right == 1:
				self.twist.angular.y = 0
				self.twist.angular.z = 0
				self.pub.publish(self.twist)
				self.check()
		else:
			if prev_mode == 1:
				#print("Stopping")
				self.twist.angular.y = 0
				self.twist.angular.z = 0
				self.pub.publish(self.twist)
				self.check()
			else:
				#print("Not stopping")
				self.check()

		# wheel speeds for reconfig
		self.reconfiguring.linear.x = self.rec_l * recon_move
		self.reconfiguring.linear.y = self.rec_r * recon_move
		self.reconfiguring.linear.z = self.rec_l * recon_move
		self.reconfiguring.angular.x = self.rec_r * recon_move
		
		# check if expanded/contracted limit
		if self.width <= self.contract_limit:
			if self.reconfiguring.linear.y >= 0 or self.reconfiguring.angular.x >= 0:
				self.reconfiguring.linear.y = 0
				self.reconfiguring.angular.x = 0

			if self.reconfiguring.linear.x <= 0 or self.reconfiguring.linear.z <= 0:
				self.reconfiguring.linear.x = 0
				self.reconfiguring.linear.z = 0

		else if self.width >= self.expand_limit:
			if self.reconfiguring.linear.y <= 0 or self.reconfiguring.angular.x <= 0:
				self.reconfiguring.linear.y = 0
				self.reconfiguring.angular.x = 0

			if self.reconfiguring.linear.x >= 0 or self.reconfiguring.linear.z >= 0:
				self.reconfiguring.linear.x = 0
				self.reconfiguring.linear.z = 0
		self.recon.publish(self.reconfiguring)

		# cmd_vel for robot
		self.twist.angular.y = f * (not self.rec_r and not self.rec_l)
		self.twist.angular.z = s * (not self.rec_r and not self.rec_l)
		self.pub.publish(self.twist)

	def e_stop(self):
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		self.pub.publish(self.twist)

	def run(self):
		b = self.custom_twist(0, self.brush.data*-100)
		self.brushes.publish(b)
		a = self.custom_twist(0, self.act.data*100)
		self.actuators.publish(a)
		v = self.custom_twist(self.vac.data*100, 0)
		self.vacuum.publish(v)
		if sum(self.input_list) != self.pub_once:
			if sum(self.input_list) > 3:
				print("Error: Pressing more than 2 buttons")
			else:
				if self.human_dist > self.human_stop:
					self.pub_once = sum(self.input_list)
					self.locomotion()
					self.print_instructions()
				else:
					self.e_stop()
		else:
			self.pub_once = sum(self.input_list)
			self.print_instructions()
			if self.human_dist <= self.human_stop:
				self.e_stop()

		#print(sum(self.input_list), self.mode)

	def print_instructions(self):
		print('\n')
		print("    MOVEMENT    ")
		print("    --------    ")
		print("[Left joystick]: Forward/Backwards")
		print("[Right joystick]: Left/Right")
		print("[left]: Rotate Left")
		print("[Right]: Rotate Right" + '\n')

		print("    HOLONOMIC/RECONFIGURATION MOVEMENT    ")
		print("    ----------------------------------    ")
		print("[r1] + [up]: Holonomic Right")
		print("[l1] + [up]: Holonomic Left")
		print("[r2] + [up]/[down]: Right Contract/Expand")
		print("[l2] + [down]/[up]: Left Contract/Expand" + '\n')
		print("[up] VISION mode: " + str(self.vision.data) + '\n')

		print("    ADJUST SPEED    ")
		print("    ------------    ")
		print("[Triangle/Cross]: + VX/WZ")
		print("[share] + [triangle/cross]: - VX/WZ" + '\n')

		print("    Current Velocity: ")
		print("    -----------------")
		print("VX: " + str(self.vx))
		print("WZ: " + str(self.wz))
		print("Robot Width: " + str(self.width) + '\n')

		print("    Cleaning:")
		print("    ---------")
		print("[options]: Brushes -> " + str(self.brush.data))
		print("[square]: Actuators -> " + str(self.act.data))
		print("[circle]: Vacuum -> " + str(self.vac.data))

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
		start.run()
		rate.sleep()
