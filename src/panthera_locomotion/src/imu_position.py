#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

x,y,z = 0,0,0

r,p,y = 0,0,0

vx,vy,vz = 0,0,0

az = 9.8

def imu_callback(msg):
	global x,y,z,r,p,y,vx,vy,vz,az,t

	ts = Twist()
	dt = rospy.get_time() - t

	a = msg.linear_acceleration.z - az
	az = msg.linear_acceleration.z
	print(az)

	# convert orientation
	orientation_q = msg.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	r, p, y = euler_from_quaternion(orientation_list)

	# get velocity
	vx = vx + msg.linear_acceleration.x * dt
	vy = vy + msg.linear_acceleration.y * dt
	vz = vz + a * dt

	# get position
	x = x + vx * dt
	y = y + vy * dt
	z = z + vz * dt

	# set t
	t = rospy.get_time()

	# publish position
	ts.linear.x = x
	ts.linear.y = y
	ts.linear.z = z

	# publish orientation
	ts.angular.x = r
	ts.angular.y = p
	ts.angular.z = y

	pub.publish(ts)

if __name__ == "__main__":
	rospy.init_node("imu_position")
	t = rospy.get_time()
	rospy.Subscriber("/imu/data", Imu, imu_callback)
	pub = rospy.Publisher("/pose", Twist, queue_size=100)
	rospy.spin()