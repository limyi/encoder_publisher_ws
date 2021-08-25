#!/usr/bin/env python
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PolygonStamped, Point32, PoseStamped, TransformStamped
from tf import TransformBroadcaster

length = 2.2
width = 0.7
right_to_lidar = 0.35
cover_offset = 0.1 # lidar offset from centre

robot_pose = PoseStamped()

def width_callback(msg):
	global width, robot_pose
	width = msg.angular.z
	
	fp = PolygonStamped()
	# footprint corners
	lb_pt = Point32()
	lb_pt.x = -length/2
	lb_pt.y = (width+2*cover_offset)/2

	lf_pt = Point32()
	lf_pt.x = length/2
	lf_pt.y = (width+2*cover_offset)/2	

	rf_pt = Point32()
	rf_pt.x = length/2
	rf_pt.y = -(width+2*cover_offset)/2

	rb_pt = Point32()
	rb_pt.x = -length/2
	rb_pt.y = -(width+2*cover_offset)/2

	fp.header.frame_id = "baselink"
	fp.header.stamp = rospy.Time.now()
	fp.polygon.points = [lb_pt, lf_pt, rf_pt, rb_pt]
	footprint_pub.publish(fp)


	br = tf2_ros.transform_broadcaster.TransformBroadcaster()
	ts = TransformStamped()
	ts2 = TransformStamped()

	# transform from velodyne to base link
	ts.header.stamp = rospy.Time.now()
	ts.header.frame_id = "base_link"
	ts.child_frame_id = "velodyne"

	ts.transform.translation.x = 0
	ts.transform.translation.y = 0#-(width/2-cover_offset) # horizontal displacement
	ts.transform.translation.z = 0

	ts.transform.rotation.x = 0
	ts.transform.rotation.y = 0
	ts.transform.rotation.z = 0
	ts.transform.rotation.w = 1


	br.sendTransform(ts)
	# static tf base_link to footprint
	ts2.header.stamp = rospy.Time.now()
	ts2.header.frame_id = "base_link"
	ts2.child_frame_id = "baselink"

	ts2.transform.translation.x = 0
	ts2.transform.translation.y = 0
	ts2.transform.translation.z = 0

	ts2.transform.rotation.x = 0
	ts2.transform.rotation.y = 0
	ts2.transform.rotation.z = 1
	ts2.transform.rotation.w = 0


	br.sendTransform(ts2)

def pose_callback(msg):
	# subscribe to robot pose
	global robot_pose
	robot_pose = msg.pose.orientation

if __name__== "__main__":
	rospy.init_node("adaptive_tf_node")
	rospy.Subscriber("/can_encoder", Twist, width_callback)
	rospy.Subscriber("/ndt_pose", PoseStamped, pose_callback)
	footprint_pub = rospy.Publisher("/footprint", PolygonStamped, queue_size=1)
	rospy.spin()
