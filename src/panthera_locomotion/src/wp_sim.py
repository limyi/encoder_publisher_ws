#!/usr/bin/env python

import rospy
import time
import tf2_ros
from tf import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped
from autoware_msgs.msg import VehicleCmd

def ndt_posestamped(msg):
	br = tf2_ros.transform_broadcaster.TransformBroadcaster()
	ts = TransformStamped()

	ts.header.stamp = rospy.Time.now()
	ts.header.frame_id = "map"
	ts.child_frame_id = "base_link"

	ts.transform.translation.x = msg.pose.position.x
	ts.transform.translation.y = msg.pose.position.y
	ts.transform.translation.z = msg.pose.position.z

	ts.transform.rotation.x = msg.pose.orientation.x
	ts.transform.rotation.y = msg.pose.orientation.y
	ts.transform.rotation.z = msg.pose.orientation.z
	ts.transform.rotation.w = msg.pose.orientation.w

	br.sendTransform(ts)

def read_cmd(msg):
	print(msg)

if __name__ == "__main__":
	try:
		rospy.init_node('wp_sim')
		rospy.Subscriber("/ndt_pose", PoseStamped, ndt_posestamped)
		rospy.Subscriber('vehicle_cmd', VehicleCmd, read_cmd)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass