#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
from serial.tools import list_ports
import struct
import time
import sys
import distance_lib as distance_lib
from local_planner.msg import Sonar

class RosHandler:

    def __init__(self):

        self.data_publisher1 = rospy.Publisher("/ultrasonic_data1", Twist, queue_size=3)
        self.data_publisher2 = rospy.Publisher("/ultrasonic_data2", Twist, queue_size=3)
        self.sonar_pub = rospy.Publisher("/ultrasonic_data", Sonar, queue_size=3)
        #Command to read address Address1:"A9FCFE73", Address2:"AAFCFE43", Address3:"ABFCFE52", Address4:"ACFCFE70", Address5:"ADFCFE61", Address6:"AEFCFE51", Address7:"AFFCFE40"
        self.address1 = "A9FCFE73"
        self.address2 = "AAFCFE43"
        self.address3 = "ABFCFE52"
        self.address4 = "ACFCFE70"
        self.address5 = "ADFCFE61"
        self.address6 = "AEFCFE51"
        self.address7 = "AFFCFE40"

        # ports 
        self.sn1 = list(serial.tools.list_ports.grep("AB0LHEK2"))#
        self.port1 = '/dev/' + self.sn1[0].name

        self.sn2 = list(serial.tools.list_ports.grep("AQ00F1QY"))#
        self.port2 = '/dev/' + self.sn2[0].name

        self.sn3 = list(serial.tools.list_ports.grep("A505HS9H"))#
        self.port3 = '/dev/' + self.sn3[0].name

        self.sn4 = list(serial.tools.list_ports.grep("AB0LHQYF"))#
        self.port4 = '/dev/' + self.sn4[0].name

        self.sn5 = list(serial.tools.list_ports.grep("AB0LH4YA"))#
        self.port5 = '/dev/' + self.sn5[0].name

        self.sn6 = list(serial.tools.list_ports.grep("AB0LHVQL"))#
        self.port6 = '/dev/' + self.sn6[0].name

        self.sn7 = list(serial.tools.list_ports.grep("AB0LHVQN"))#
        self.port7 = '/dev/' + self.sn7[0].name

        self.sn8 = list(serial.tools.list_ports.grep("AB0LH5DD"))#
        self.port8 = '/dev/' + self.sn8[0].name

        self.sn9 = list(serial.tools.list_ports.grep("AB0LH4YC"))
        self.port9 = '/dev/' + self.sn9[0].name

        self.sn10 = list(serial.tools.list_ports.grep("AB0LHVQ1"))#
        self.port10 = '/dev/' + self.sn10[0].name
	   #"AI06B90T" "A505HS9H""AQ00F1QY"   "A505HS1X"   				#Examples of USB Serial Number. Linux Command: usb-devices to get USB Information and Serial Number
        
        self.Panthera_ultrasonic_1  = distance_lib.dis(self.port1, self.address1)
        self.Panthera_ultrasonic_2  = distance_lib.dis(self.port2, self.address3)
        self.Panthera_ultrasonic_3  = distance_lib.dis(self.port3, self.address3)
        self.Panthera_ultrasonic_4  = distance_lib.dis(self.port4, self.address5)
        self.Panthera_ultrasonic_5  = distance_lib.dis(self.port5, self.address5)
        self.Panthera_ultrasonic_6  = distance_lib.dis(self.port6, self.address2)
        self.Panthera_ultrasonic_7  = distance_lib.dis(self.port7, self.address4)
        self.Panthera_ultrasonic_8  = distance_lib.dis(self.port8, self.address6)
        self.Panthera_ultrasonic_9  = distance_lib.dis(self.port9, self.address2)
        self.Panthera_ultrasonic_10 = distance_lib.dis(self.port10, self.address6)

    def run(self):

        while not rospy.is_shutdown():

            sonar = Sonar()
           
            distance_1 = self.Panthera_ultrasonic_1.readDistanceInfo()
            distance_2 = self.Panthera_ultrasonic_2.readDistanceInfo()
            distance_3 = self.Panthera_ultrasonic_3.readDistanceInfo()  
            distance_4 = self.Panthera_ultrasonic_4.readDistanceInfo()
            distance_5 = self.Panthera_ultrasonic_5.readDistanceInfo()
            distance_6 = self.Panthera_ultrasonic_6.readDistanceInfo()
            distance_7 = self.Panthera_ultrasonic_7.readDistanceInfo()  
            distance_8 = self.Panthera_ultrasonic_8.readDistanceInfo()
            distance_9 = self.Panthera_ultrasonic_9.readDistanceInfo()
            distance_10 = self.Panthera_ultrasonic_10.readDistanceInfo()

            sonar.back_l = distance_1
            sonar.back_r = distance_2

            sonar.left_b = distance_3
            sonar.left_m = distance_4
            sonar.left_f = distance_5

            sonar.front_l = distance_6
            sonar.front_r = distance_7

            sonar.right_b = distance_8
            sonar.right_m = distance_9
            sonar.right_f = distance_10

            self.sonar_pub.publish(sonar)
            '''
            data_array1 = Twist()
            data_array2 = Twist()

            data_array1.linear.x  = distance_1
            data_array1.linear.y  = distance_2
            data_array1.linear.z  = distance_3
            data_array1.angular.x = distance_4
            data_array1.angular.y = distance_5
            data_array1.angular.z = 0

            data_array2.linear.x  = distance_6
            data_array2.linear.y  = distance_7
            data_array2.linear.z  = distance_8
            data_array2.angular.x = distance_9
            data_array2.angular.y = distance_10
            data_array2.angular.z = 0

		
	

            self.data_publisher1.publish(data_array1)
            self.data_publisher2.publish(data_array2)
            '''



            rospy.sleep(0.1)
        return 

if __name__ == '__main__':
    rospy.init_node('ultrasonic_node')
    rate = rospy.Rate(10) # 10hz
    rh = RosHandler()
    rh.run()
