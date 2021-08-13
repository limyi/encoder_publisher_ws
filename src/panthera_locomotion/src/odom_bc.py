#!/usr/bin/env python

import rospy
import time
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster

def tf_br(msg):
	# transform odom to base link
	br = tf2_ros.transform_broadcaster.TransformBroadcaster()
	ts = TransformStamped()

	ts.header.stamp = rospy.Time.now()
	ts.header.frame_id = "odom"
	ts.child_frame_id = "base_link"

	ts.transform.translation.x = msg.pose.pose.position.x
	ts.transform.translation.y = msg.pose.pose.position.y
	ts.transform.translation.z = 0

	ts.transform.rotation.x = msg.pose.pose.orientation.x
	ts.transform.rotation.y = msg.pose.pose.orientation.y
	ts.transform.rotation.z = msg.pose.pose.orientation.z
	ts.transform.rotation.w = msg.pose.pose.orientation.w

	br.sendTransform(ts)

if __name__ == "__main__":
	try:
		rospy.init_node("base_link_tf")
		rospy.Subscriber("/panthera_odom", Odometry, tf_br)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass



