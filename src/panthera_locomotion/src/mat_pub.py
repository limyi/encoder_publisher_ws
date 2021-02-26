#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

method = 2
width = 0.68
min_width = 0.7
max_width = 1.0

mat1 = Float64MultiArray()
mat2 = Float64MultiArray()
mat3 = Float64MultiArray()
mat1.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
mat2.data = [7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
mat3.data = [0, 0, 0, 0, 0, 0]


def reconfig(data):
	global width
	width = (data.angular.y + data.angular.z)/2

def kinematics_matrix1(w):
	if method == 1:
		if w <= min_width:
			pub.publish(mat1)

		elif w >= max_width:
			pub.publish(mat2)
	elif method == 2:
		if w <= min_width:
			pub.publish(mat1)

		elif w >= max_width:
			for i in range(len(mat1.data)):
				mat3.data[i] = mat2.data[i] - mat1.data[i]
			pub.publish(mat3)

if __name__ == "__main__":
	rospy.init_node("matrix_pub")
	pub = rospy.Publisher('t_matrix', Float64MultiArray, queue_size=1)
	rospy.Subscriber('can_encoder', Twist, reconfig)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		kinematics_matrix1(width)
		rate.sleep()

