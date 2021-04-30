#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, PoseWithCovarianceStamped

initial_pose = PoseWithCovarianceStamped()
time_just_now = 0

time_now = 0
received_pose = False

def init_pose(msg):
	initialpose = msg.pose.pose
	print(initialpose)
	received_pose = True
	time_just_now = rospy.get_time()

def cmd_vel(msg):

	if received_pose == True:
		curr_pose = PoseStamped()
		time_now = rospy.get_time()
		dt = time_now - time_just_now
		br = tf2_ros.transform_broadcaster.TransformBroadcaster()
		ts = TransformStamped()

		ts.header.stamp = rospy.Time.now()
		ts.header.frame_id = "map"
		ts.child_frame_id = "base_link"

		ts.transform.translation.x = initial_pose.pose.pose.position.x + msg.linear.x * dt
		curr_pose.pose.position.x = ts.transform.translation.x

		ts.transform.translation.y = initial_pose.pose.pose.position.y + msg.linear.y * dt
		curr_pose.pose.position.y = ts.transform.translation.y

		ts.transform.translation.z = 0
		curr_pose.pose.position.z = ts.transform.translation.x

		euler = tf.transformations.euler_from_quaternion(initial_pose.pose.pose.orientation)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		yaw += msg.angular.z * dt

		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		curr_pose.pose.orientation.x = quaternion[0]
		curr_pose.pose.orientation.y = quaternion[1]
		curr_pose.pose.orientation.z = quaternion[2]
		curr_pose.pose.orientation.w = quaternion[3]

		ts.transform.rotation.x = quaternion[0]
		ts.transform.rotation.y = quaternion[1]
		ts.transform.rotation.z = quaternion[2]
		ts.transform.rotation.w = quaternion[3]

		br.sendTransform(ts)
		ndt_pub.publish(curr_pose)
		time_just_now = time_now

if __name__ == "__main__":
	rospy.init_node("sim_panthera")
	rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, init_pose)
	rospy.Subscriber("/panthera_cmd", Twist, cmd_vel)
	ndt_pub = rospy.Publisher("/ndt_pose", PoseStamped, queue_size=100)
	rospy.spin()