#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf

class RobotOdom():
	def __init__(self):
		#rospy.init_node('robot_odom')
		rospy.Subscriber('/lb_wheel_vel', Float32, self.lb_lin_vel)
		rospy.Subscriber('/lf_wheel_vel', Float32, self.lf_lin_vel)
		rospy.Subscriber('/rb_wheel_vel', Float32, self.rb_lin_vel)
		rospy.Subscriber('/rf_wheel_vel', Float32, self.rf_lin_vel)
		rospy.Subscriber('/encoder_positions', Twist, self.wheel_encoders)
		self.odom_pub = rospy.Publisher('/panthera_odom', Odometry, queue_size=10)

		self.lb_lin = 0
		self.lf_lin = 0
		self.rb_lin = 0
		self.rf_lin = 0

		self.lb_steer = 0
		self.lf_steer = 0
		self.rb_steer = 0
		self.rf_steer = 0

		self.length = 1.31
		self.width = 0.89
		self.wheel_radius = 0.1

		self.left_x_dot = 0
		self.left_y_dot = 0
		self.left_theta_dot = 0
		self.right_x_dot = 0
		self.right_y_dot = 0
		self.right_theta_dot = 0

		self.x_dot = 0
		self.y_dot = 0
		self.theta_dot = 0

		self.right_x = 0
		self.right_y = self.width/2
		self.right_theta = 0
		self.left_x = 0
		self.left_y = -self.width/2
		self.left_theta = 0

		self.x_pos = 0
		self.y_pos = 0
		self.phi = math.pi / 2

		self.global_x = 0
		self.global_y = 0
		self.global_theta = 0
		self.global_x_dot = 0
		self.global_y_dot = 0
		self.global_theta_dot = 0

	def lb_lin_vel(self, data):
		self.lb_lin = data.data

	def lf_lin_vel(self, data):
		self.lf_lin = data.data

	def rb_lin_vel(self, data):
		self.rb_lin = data.data

	def rf_lin_vel(self, data):
		self.rf_lin = data.data

	def wheel_encoders(self, data):
		self.lb_steer = math.radians(data.linear.x)
		self.rb_steer = math.radians(data.linear.y)
		self.lf_steer = math.radians(data.linear.z)
		self.rf_steer = math.radians(data.angular.x)

	def left_vel(self):
		#self.left_x_dot = (self.lf_lin*math.cos(self.lf_steer)*math.cos(self.phi) + self.lb_lin*math.cos(self.lb_steer)*math.cos(self.phi))/2
		#self.left_y_dot = (self.lf_lin*math.sin(self.lf_steer)*math.cos(self.phi) + self.lb_lin*math.sin(self.lb_steer)*math.cos(self.phi))/2
		self.left_x_dot = (self.lf_lin*math.cos(self.lf_steer) + self.lb_lin*math.cos(self.lb_steer))/2
		self.left_y_dot = (self.lf_lin*math.sin(self.lf_steer) + self.lb_lin*math.sin(self.lb_steer))/2
		v = (self.lf_lin*math.cos(self.lf_steer) + self.lb_lin*math.cos(self.lb_steer))/2
		try:
			r = (self.length / 2) / math.tan(self.lf_steer)
		except ZeroDivisionError:
			r = float('inf')
		self.left_theta_dot = round(v/r,3)

	def right_vel(self):
		#self.right_x_dot = (self.rf_lin*math.cos(self.rf_steer)*math.cos(self.phi) + self.rb_lin*math.cos(self.rb_steer)*math.cos(self.phi))/2
		#self.right_y_dot = (self.rf_lin*math.sin(self.rf_steer)*math.cos(self.phi) + self.rb_lin*math.sin(self.rb_steer)*math.cos(self.phi))/2
		self.right_x_dot = (self.rf_lin*math.cos(self.rf_steer) + self.rb_lin*math.cos(self.rb_steer))/2
		self.right_y_dot = (self.rf_lin*math.sin(self.rf_steer) + self.rb_lin*math.sin(self.rb_steer))/2
		v = (self.rf_lin*math.cos(self.rf_steer) + self.rb_lin*math.cos(self.rb_steer))/2
		try:
			r = (self.length / 2) / math.tan(self.rf_steer)
		except ZeroDivisionError:
			r = float('inf')
		self.right_theta_dot = round(v/r,3)

	def calc_robot_vel(self):
		self.left_vel()
		self.right_vel()
		self.x_dot = (self.left_x_dot + self.right_x_dot) / 2
		self.y_dot = (self.left_y_dot + self.right_y_dot) / 2
		self.theta_dot = (self.left_theta_dot + self.right_theta_dot) / 2

	def right_pos(self, dt):
		self.right_x += self.right_x_dot * dt
		self.right_y += self.right_y_dot * dt
		self.right_theta += self.right_theta_dot * dt 

	def left_pos(self, dt):
		self.left_x += self.left_x_dot * dt
		self.left_y += self.left_y_dot * dt
		self.left_theta += self.left_theta_dot * dt

	def calc_robot_pos(self, dt):
		self.right_pos(dt)
		self.left_pos(dt)
		self.x_pos = (self.right_x + self.left_x) / 2
		self.y_pos = (self.right_y + self.left_y) / 2
		avg_theta = math.modf((self.right_theta + self.left_theta) / 2)
		if avg_theta[1]%2 == 0:
			if avg_theta[1] >= 0:
				self.theta_pos = avg_theta[0] * math.pi
			else:
				self.theta_pos = avg_theta[0] * -math.pi
		else:
			if avg_theta[1] >= 0:
				self.theta_pos = (1-avg_theta[0]) * -math.pi
			else:
				self.theta_pos = (1+avg_theta[0]) * math.pi

		self.phi += round(self.theta_pos,3)


	def global_vel(self):
		rot_mat = np.array([[math.cos(self.phi), math.sin(self.phi), 0],
							[-math.sin(self.phi), math.cos(self.phi), 0],
							[0, 0, 1]])
		robot_vel_mat = np.array([[self.x_dot],
								  [self.y_dot],
								  [self.theta_dot]])
		global_vel_mat = np.matmul(rot_mat,robot_vel_mat)
		self.global_x_dot = global_vel_mat[0]
		self.global_y_dot = global_vel_mat[1]
		self.global_theta_dot = global_vel_mat[2]

	def global_pos(self):
		rot_mat = np.array([[math.cos(self.phi), math.sin(self.phi), 0],
							[-math.sin(self.phi), math.cos(self.phi), 0],
							[0, 0, 1]])
		robot_pos_mat = np.array([[self.x_pos],
								  [self.y_pos],
								  [self.theta_pos]])
		global_pos_mat = np.matmul(rot_mat,robot_pos_mat)
		self.global_x = global_pos_mat[0]
		self.global_y = global_pos_mat[1]
		self.global_theta = global_pos_mat[2]

	def odom_loop(self,dt):
		#dt = 0.2
		#now = rospy.get_time()
		#rate = rospy.Rate(20)
		self.calc_robot_vel()
		self.calc_robot_pos(dt)
		self.global_vel()
		self.global_pos()
		#print("Calculation done")
		odom = Odometry()
		
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom.header.stamp = rospy.Time.now()

		odom.pose.pose.position.x = self.x_pos#self.global_x
		odom.pose.pose.position.y = self.y_pos#self.global_y
		odom.pose.pose.position.z = 0
		#print("odom position")
		odom_quat = Quaternion()
		odom_quat = tf.transformations.quaternion_from_euler(0,0,self.global_theta)
		odom.pose.pose.orientation = Quaternion(*odom_quat)
		#print("Quat done")
		odom.twist.twist.linear.x = self.global_x_dot
		odom.twist.twist.linear.y = self.global_y_dot
		odom.twist.twist.angular.z = self.global_theta_dot
		#later = rospy.get_time()
		#dt = later - now
		#print(dt)
		#print(odom)
		self.odom_pub.publish(odom)
	        #rate.sleep()

if __name__ == "__main__":
	try:
		rospy.init_node('robot_odom')
		robot_odom = RobotOdom()
		rate = rospy.Rate(30)
		dt = 0
		now = rospy.get_time()
		while not rospy.is_shutdown():
			robot_odom.odom_loop(dt)
			rate.sleep()
			later = rospy.get_time()
			dt = later-now
			now = later
	except rospy.ROSInterruptException:
		pass