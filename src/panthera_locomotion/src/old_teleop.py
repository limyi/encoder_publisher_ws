#!/usr/bin/env python

import math
import rospy
import serial
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Empty
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse
import serial.tools.list_ports as lp

width = 0.89
length = 1.31

lb = 0
lf = 0
rb = 0
rf = 0

pos_lb = 0
pos_rb = 0
pos_lf = 0
pos_rf = 0

linear_x = 0.025
angular_z = 0.05618

contract_vel = 0.1

tolerance = 1.0

distance = 0.55
x_dist = 0
z_dist = 2
state = 66

max_width = 1.0
min_width = 0.7
test_mode = True

ct = 4
####### Bluetooth port #######
p = list(lp.grep('AQ00F1QY'))
port = '/dev/' + p[0].name
##############################

def human_loc(msg):
	global x_dist, z_dist
	x_dist = msg.x
	z_dist = msg.z
	
def state_machine():
	global state
	cmd = 'hello'
	if z_dist <= distance:
		if state == 0:
			if x_dist <= 0:
				cmd = 'j' # Contract right
				state = 1
			else:
				cmd = 'g' # Contract left
				state = 2
		elif state == 66:
			cmd = 's'
			state = 0
		else:
			pass

	else:
		if state == 0:
			cmd = 's'
		elif state == 1:
			cmd = 'u' # Expand right
			state = 0
		elif state == 2:
			cmd = 't' # Expand left
			state = 0
			
		elif state == 66:
			cmd = 's'
			state = 0
	return cmd
		

def adjust_wheels(vx, wz): # radius in m, direction c(-1) or ccw(1)
	global lb,lf,rb,rf
	if wz == 0:
		radius = float('inf')
	else:
		radius = vx/wz
	left = radius - width/2
	right = radius + width/2
	
	lf = round(math.degrees(math.atan((length*0.5) / left)), 2)
	rf = round(math.degrees(math.atan((length*0.5) / right)), 2)
	lb = -lf
	rb = -rf

def enc_pos(data):
	global pos_lb, pos_lf, pos_rb, pos_rf, width
	pos_lb = data.linear.x
	pos_rb = data.linear.y
	pos_lf = data.linear.z
	pos_rf = data.angular.x
	width = (data.angular.y + data.angular.z)/2

def check():
	req = StatusRequest()
	req.reconfig = True
	signal = False
	rate = rospy.Rate(2)
	while signal == False and not rospy.is_shutdown():
		rate.sleep()
		lb = lb_status(req)
		rb = rb_status(req)
		lf = lf_status(req)
		rf = rf_status(req)
		signal = (lb.status and rb.status and lf.status and rf.status)
		print([lb.status, rb.status, lf.status, rf.status])
		print("Status of steering motors:" + str(signal))
		
def reconfig(state):
	req = StatusRequest()
	req.reconfig = state
	stat = not state
	while stat!= state:
		lb = lb_stat(req)
		lf = lf_stat(req)
		rb = rb_stat(req)
		rf = rf_stat(req)
		stat = (lb.status and rb.status and lf.status and rf.status)

def reconfig_angle(vx, vy):
	angle = round(math.atan(vy/vx), 2)
	angle = angle/(2*math.pi) * 360
	speed = math.sqrt(vx**2 + vy**2)
	print(vx,vy)
	return angle, speed

def run(x):
	global linear_x
	global angular_z
	global contract_vel

	twist = Twist()
	twist2 = Twist()

	### MOVEMENT ###

	if x == 'w':
		adjust_wheels(linear_x, 0)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = 0
		pub.publish(twist)

	elif x == 'e':
		adjust_wheels(linear_x, -angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = -angular_z
		pub.publish(twist)

	elif x == 'q':
		adjust_wheels(linear_x, angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = angular_z
		pub.publish(twist)

	elif x == 'd':
		adjust_wheels(0, -angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = 0
		twist.angular.z = -angular_z
		pub.publish(twist)

	elif x == 'a':
		adjust_wheels(0, angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = 0
		twist.angular.z = angular_z
		pub.publish(twist)

	elif x == 's':
		adjust_wheels(0, 0)
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		recon.publish(twist2)

	elif x == 'c':
		adjust_wheels(0, 0)
		twist.linear.x = -90
		twist.linear.y = -90
		twist.linear.z = -90
		twist.angular.x = -90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = 0
		pub.publish(twist)

	elif x == 'z':
		adjust_wheels(0, 0)
		twist.linear.x = 90
		twist.linear.y = 90
		twist.linear.z = 90
		twist.angular.x = 90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = 0
		pub.publish(twist)

	elif x == 'r':
		adjust_wheels(0, 0)
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = -linear_x
		twist.angular.z = 0
		pub.publish(twist)

	################ RECONFIGURATION EXPANSION #####################

	elif x == 't': # Expand left
		adjust_wheels(0, 0)
		twist.linear.x = 90
		twist.linear.y = 0
		twist.linear.z = 90
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)

		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = linear_x
		twist2.linear.z = linear_x
		recon.publish(twist2)
		
		if test_mode == True:
			rospy.sleep(ct) # Adjust time for expansion length
		else:
			while width < max_width:
				print("Expanding")
	
		twist2.linear.x = 0
		twist2.linear.z = 0
		recon.publish(twist2)
		
	elif x == 'u': # Expand right
		adjust_wheels(0, 0)
		twist.linear.x = 0
		twist.linear.y = 90
		twist.linear.z = 0
		twist.angular.x = 90
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.y = -linear_x
		twist2.angular.x = -linear_x
		recon.publish(twist2)
		
		if test_mode == True:
			rospy.sleep(ct) # Adjust time for expansion length
		else:
			while width < max_width:
				print("Expanding")
		
		twist2.linear.y = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	elif x == 'y': # Expand both sides
		adjust_wheels(0, 0)
		twist.linear.x = 90
		twist.linear.y = -90
		twist.linear.z = 90
		twist.angular.x = -90
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = linear_x
		twist2.linear.y = linear_x
		twist2.linear.z = linear_x
		twist2.angular.x = linear_x
		recon.publish(twist2)
		
		if test_mode == True:
			rospy.sleep(ct/2) # Adjust time for expansion length
		else:
			while width < max_width:
				print("Expanding")

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	# RECONFIGURATION CONTRACTION

	elif x == 'g': # Contract left
		adjust_wheels(0, 0)
		twist.linear.x = 90
		twist.linear.y = 0
		twist.linear.z = 90
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = -linear_x
		twist2.linear.z = -linear_x
		recon.publish(twist2)
		
		if test_mode == True:
			rospy.sleep(ct) # Adjust time for contraction length
		else:
			while width > min_width:
				print("Contracting")

		twist2.linear.x = 0
		twist2.linear.z = 0
		recon.publish(twist2)
		
	elif x == 'j': # Contract right
		adjust_wheels(0, 0)
		twist.linear.x = 0
		twist.linear.y = 90
		twist.linear.z = 0
		twist.angular.x = 90
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		pub.publish(twist)
		recon.publish(twist2)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.y = linear_x
		twist2.angular.x = linear_x
		recon.publish(twist2)
		
		if test_mode == True:
			rospy.sleep(ct) # Adjust time for contraction length
		else:
			while width > min_width:
				print("Contracting")
		
		twist2.linear.y = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	elif x == 'h': # Contract both sides
		adjust_wheels(0, 0)
		twist.linear.x = -90
		twist.linear.y = 90
		twist.linear.z = -90
		twist.angular.x = 90
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = linear_x
		twist2.linear.y = linear_x
		twist2.linear.z = linear_x
		twist2.angular.x = linear_x
		recon.publish(twist2)
		
		if test_mode == True:
			rospy.sleep(ct/2) # Adjust time for contraction length
		else:
			while width > min_width:
				print("Contracting")

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	############## DYNAMIC RECONFIGURATION ###################

	elif x == '1': # Contract Left
		adjust_wheels(0, 0)
		angle, speed = reconfig_angle(linear_x,contract_vel)
		twist.linear.x = 0
		twist.linear.y = angle
		twist.linear.z = 0
		twist.angular.x = angle
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = -linear_x
		twist2.linear.y = -speed
		twist2.linear.z = -linear_x
		twist2.angular.x = -speed
		recon.publish(twist2)
		
		rospy.sleep(1) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	elif x == '2': # Contract Right
		adjust_wheels(0, 0)
		angle, speed = reconfig_angle(linear_x,-contract_vel)
		twist.linear.x = angle
		twist.linear.y = 0
		twist.linear.z = angle
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = -speed
		twist2.linear.y = -linear_x
		twist2.linear.z = -speed
		twist2.angular.x = -linear_x
		recon.publish(twist2)
		
		rospy.sleep(1) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	elif x == '3': # Contract Centre
		adjust_wheels(0, 0)
		angle, speed = reconfig_angle(linear_x, contract_vel/2)
		twist.linear.x = -angle
		twist.linear.y = angle
		twist.linear.z = -angle
		twist.angular.x = angle
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = -speed
		twist2.linear.y = -speed
		twist2.linear.z = -speed
		twist2.angular.x = -speed
		recon.publish(twist2)
		
		rospy.sleep(1) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	elif x == '4': # Expand Right
		adjust_wheels(0, 0)
		angle, speed = reconfig_angle(linear_x, -contract_vel)
		twist.linear.x = 0
		twist.linear.y = angle
		twist.linear.z = 0
		twist.angular.x = angle
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = -linear_x
		twist2.linear.y = -speed
		twist2.linear.z = -linear_x
		twist2.angular.x = -speed
		recon.publish(twist2)
		
		rospy.sleep(1) # Adjust time for expansion length
		
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	elif x == '5': # Expand Left
		adjust_wheels(0, 0)
		angle, speed = reconfig_angle(linear_x, contract_vel)
		twist.linear.x = 0
		twist.linear.y = angle
		twist.linear.z = 0
		twist.angular.x = angle
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = -speed
		twist2.linear.y = -linear_x
		twist2.linear.z = -speed
		twist2.angular.x = -linear_x
		recon.publish(twist2)
		
		rospy.sleep(1) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	elif x == '6': # Contract Centre
		adjust_wheels(0, 0)
		angle, speed = reconfig_angle(linear_x, contract_vel/2)
		twist.linear.x = angle
		twist.linear.y = -angle
		twist.linear.z = angle
		twist.angular.x = -angle
		twist.angular.y = 0
		twist.angular.z = 0
		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = -speed
		twist2.linear.y = -speed
		twist2.linear.z = -speed
		twist2.angular.x = -speed
		recon.publish(twist2)
		
		rospy.sleep(1) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)
		
	# ADJUST VX AND WZ DEFAULT SPEEDS

	elif x == 'o':
		linear_x += 0.05

	elif x == 'p':
		angular_z += 0.05

	elif x == 'n':
		linear_x -= 0.05

	elif x == 'm':
		angular_z -= 0.05
	'''
	print("\nvx: " + str(linear_x) + "m/s")
	print("wz: " + str(angular_z) + "rad/s")
	print("Turning Radius: " + str(linear_x/angular_z) + "m\n")
	'''
if __name__ == "__main__":
	try:
		rospy.init_node('controller')
		ser = serial.Serial(port)
		sub = rospy.Subscriber('encoder_positions', Twist, enc_pos)
		pub = rospy.Publisher('target_angle', Twist, queue_size=1)
		recon = rospy.Publisher('reconfig', Twist, queue_size=1)
		h_loc = rospy.Subscriber('/human_loc', Point32, human_loc)

		rospy.wait_for_service('/lb_steer_status')
		rospy.wait_for_service('/lf_steer_status')
		rospy.wait_for_service('/rb_steer_status')
		rospy.wait_for_service('/rf_steer_status')

		lb_status = rospy.ServiceProxy('lb_steer_status', Status)
		lf_status = rospy.ServiceProxy('lf_steer_status', Status)
		rb_status = rospy.ServiceProxy('rb_steer_status', Status)
		rf_status = rospy.ServiceProxy('rf_steer_status', Status)
		'''
		rospy.wait_for_service('/lb_reconfig_status')
		rospy.wait_for_service('/lf_reconfig_status')
		rospy.wait_for_service('/rb_reconfig_status')
		rospy.wait_for_service('/rf_reconfig_status')

		lb_stat = rospy.ServiceProxy('lb_reconfig_status', Status)
		lf_stat = rospy.ServiceProxy('lf_reconfig_status', Status)
		rb_stat = rospy.ServiceProxy('rb_reconfig_status', Status)
		rf_stat = rospy.ServiceProxy('rf_reconfig_status', Status)
		'''
		switch = True
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			'''
			print("w: Forward")
			print("r: Reverse")
			print("e: Turn Right with a radius vx/wz")
			print("q: Turn Left with a radius vx/wz")
			print("d/a: L/R On the spot rotation")
			print("s: Align wheels to 0 and stop")
			print("o/p: increase vx and wz by +0.05")
			print("n/m: decrease vx and wz by -0.05")
			print("c: 90 Degrees right")
			print("z: 90 Degrees left")
			print("t: Expand Left")
			print("u: Expand Right")
			print("y: Expand Both Sides")
			print("g: Contract Left")
			print("j: Contract Right")
			print("h: Contract Both Sides")
			print("1: Dynamic Contract to Left")
			print("2: Dynamic Contract to Right")
			print("3: Dynamic Contract to Centre")
			print("4: Dynamic Expand to Right")
			print("5: Dynamic Expand to Left")
			print("6: Dynamic Expand from Centre")
			cmd = ser.readline().strip('\r\n')
			print("\n" + cmd)
			'''
			cmd = state_machine()
			#reconfig(True)
			print(cmd, state)
			#print(x_dist, z_dist)
			if cmd == ' ':
				switch = False
			elif cmd == 'hello':
				pass
			else:
				run(cmd)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass


