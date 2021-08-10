#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist

def read_twist(client, userdata, message):
	twist_str = message.payload.split(",")
	twist = Twist()
	twist.linear.x = float(twist_str[0])
	twist.angular.z = float(twist_str[-1])
	pub.publish(twist)
	print(rospy.get_time())

if __name__=="__main__":
	rospy.init_node("mqtt_server")
	pub = rospy.Publisher("/panthera_cmd", Twist, queue_size=1)

	broker_address = "192.168.1.93"
	client = mqtt.Client("mqtt_server")
	client.on_message = read_twist
	client.connect(broker_address)
	client.loop_start()
	client.subscribe("cmd_vel")
	rospy.spin()