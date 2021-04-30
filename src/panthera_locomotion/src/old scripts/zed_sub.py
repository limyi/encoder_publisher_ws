#!/usr/bin/env python
import rospy
from zed_interfaces.msg import ObjectsStamped as OS

def img_callback(msg):
	min = 20
	for person in msg.objects:
		if person.position[0] <= min:
			min = person.position[0]

	print("Nearest human: " + str(min))


if __name__ == "__main__":
	rospy.init_node("zed_sub")
	rospy.Subscriber("/zed2/zed_node/obj_det/objects", OS, img_callback)
	rospy.spin()