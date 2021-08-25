#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

tw = Twist()
tw.angular.z = 1.2
tw.angular.y = 1.2
d_min = 0.8
d_max = 1.5
d = 1.2

def reconfig_callback(msg):
    global d
    if msg.data == -1:
        if d >= d_min:
            d -= 0.002
    tw.angular.z = d
    tw.angular.y = d
    pub.publish(tw)


if __name__ == '__main__':
 	rospy.init_node("fake_width")
 	pub = rospy.Publisher("/can_encoder", Twist, queue_size=1)
 	rospy.Subscriber("/reconfiguration", Float64, reconfig_callback)
 	rospy.spin()