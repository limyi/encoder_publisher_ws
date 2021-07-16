#!/usr/bin/env python
import rospy
from motor import RoboteqMotor as rm
import serial.tools.list_ports
from std_msgs.msg import Int64

def run_cmd(msg):
	motor.writeSpeed(msg.data)


def queries(msg):
	if msg.data == 0:
		mode = 0
		motor.set_mode(msg.data)
	else:
		mode = 1
		motor.set_mode(msg.data)
		motor.writeTorque(1000)
	print("mode: {}".format(str(mode)))


if __name__=="__main__":
	rospy.init_node("roboteq_motor")
	motor = rm("SBL1XXX")
	rospy.Subscriber("/run_motor", Int64, run_cmd)
	rospy.Subscriber("/query", Int64, queries)
	mode = 0
	rospy.spin()