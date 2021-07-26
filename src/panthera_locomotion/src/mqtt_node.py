#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist

def pub_cmd(msg):
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		client.publish("cmd_vel", msg)
		print(rospy.get_time())
		rate.sleep()

if __name__=="__main__":
	rospy.init_node("mqtt_client")
	broker_address = "192.168.1.93"
	client = mqtt.Client("P1")
	client.connect(broker_address)
	twist = Twist()
	twist.linear.x = 1.0
	twist.angular.z = 1.0
	ls = "1.0,0,0,0,0,1.0"
	pub_cmd(ls)
	rospy.spin()


