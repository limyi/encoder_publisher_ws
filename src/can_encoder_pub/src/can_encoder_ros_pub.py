#!/usr/bin/env python3

import rospy
import time
import can

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

#dic_init = {}

def initialise(dic):

    bustype = "socketcan"
    can_interface = "can0"

    bus = can.interface.Bus(can_interface, bustype = bustype)

    out_1 = bus.recv(1)
    out_2 = bus.recv(1)
    out_3 = bus.recv(1)
    out_4 = bus.recv(1)

        
        
    ls_out = [out_1, out_2, out_3, out_4]
    ls_id = [385, 386, 387, 389]
    dic = {}

    for i in ls_id:
        dic[i] = -1
        
    for i in ls_out:
        for j in ls_id:
            if i.arbitration_id == j:
                position = i.data[:4]
                position_value = int.from_bytes(position, byteorder = "little", signed = False)
                dic[j] = position_value

    return dic
        

def encode_pub(dic):
    pub = rospy.Publisher("encoder_positions" , Twist, queue_size = 10) #topic name
    pub2 = rospy.Publisher("encoder_speeds", Twist, queue_size = 10)
    #twist_pub = rospy.Publisher("encoder_positions", Twist, queue_size = 10) #topic name

    rospy.init_node('encode_pub', anonymous = True) #node name
    rate = rospy.Rate(10)
    
    bustype = "socketcan"
    can_interface = "can0"

    bus = can.interface.Bus(can_interface, bustype = bustype)
    

    while not rospy.is_shutdown():
        out_1 = bus.recv(1)
        out_2 = bus.recv(1)
        out_3 = bus.recv(1)
        out_4 = bus.recv(1)
        
        out_5 = bus.recv(1)
        out_6 = bus.recv(1)
        out_7 = bus.recv(1)
        out_8 = bus.recv(1)
        out_9 = bus.recv(1)
        out_10 = bus.recv(1)
        out_11 = bus.recv(1)
        out_12 = bus.recv(1)
        '''
        out_13 = bus.recv(1)
        out_14 = bus.recv(1)
        out_15 = bus.recv(1)
        out_16 = bus.recv(1)
        out_17 = bus.recv(1)
        out_18 = bus.recv(1)
        out_19 = bus.recv(1)
        out_20 = bus.recv(1)
        out_21 = bus.recv(1)
        out_22 = bus.recv(1)
        out_23 = bus.recv(1)
        out_24 = bus.recv(1)
        '''
        encoder_pos = Twist()
        encoder_speed = Twist()
        
        ls_out = [out_1, out_2, out_3, out_4, out_5, out_6, out_7, out_8, out_9, out_10, out_11, out_12]
        ls_id = [385, 386, 387, 389]
        dic_pos = {}
        dic_speed = {}

        for i in ls_id:
            dic_pos[i] = -1
            dic_speed[i] = 1
        
        for i in ls_out:
            for j in ls_id:
                if i.arbitration_id == j:
                    #print(i)
                    position = i.data[:4]
                    speed = i.data[4:6]
                    #print(speed)
                    position_value = int.from_bytes(position, byteorder = "little", signed = False)
                    speed_value = int.from_bytes(speed, byteorder = "little", signed = False)
                    #print("speed is " + str(speed_value))
                    dic_speed[j] = speed_value
                    dic_pos[j] = position_value
        
        '''
        encoder_pos.linear.x = (dic_pos[385]-67063944)*0.02197         #/16384x 2 x 180
        encoder_pos.linear.y = (dic_pos[386]-67063696)*0.02197 
        encoder_pos.linear.z = (dic_pos[387]-241763)*0.02197
        encoder_pos.angular.x = (dic_pos[388])*0.02197
        '''
        encoder_pos.linear.x = round((dic_pos[385] - 67063944)*0.02197, 2)         #/16384x 2 x 180
        encoder_pos.linear.y = round((dic_pos[386] - 137458)*0.02197, 2)
        encoder_pos.linear.z = round((dic_pos[387] - 241754)*0.02197, 2)
        encoder_pos.angular.x = round((dic_pos[389] - 13977)*0.02197, 2)
        #encoder_pos.angular.y = (dic_pos[388])#*0.02197

        encoder_speed.linear.x = dic_speed[385]
        encoder_speed.linear.y = dic_speed[386]
        encoder_speed.linear.z = dic_speed[387]
        encoder_speed.angular.x = dic_speed[389]
        '''
        encoder_speed.linear.x = dic_speed[385]
        data_1 = out_1.data
        position = data[:4]
        speed = data[4:]

        position_value = int.from_bytes(position,byteorder = 'little' , signed = False)
        speed_value = int.from_bytes(speed, byteorder = 'little' , signed = False)

        rospy.loginfo(position_value)
        pub.publish(position_value)
        '''

        rospy.loginfo(encoder_pos)
        pub.publish(encoder_pos)
        pub2.publish(encoder_speed)
if __name__ == '__main__':
    try:
        dic_init = {}
        dic_init = initialise(dic_init)
        encode_pub(dic_init)

    except rospy.ROSInterruptException:
        pass

