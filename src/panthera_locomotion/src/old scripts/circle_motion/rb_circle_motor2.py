#!/usr/bin/env python

import rospy
import time
import orienbus
import serial.tools.list_ports
import math

from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusResponse

# PID parameters
kp = 100#rospy.get_param('/rb_pid')['kp']
ki = 0#rospy.get_param('/rb_pid')['ki']
kd = 0#rospy.get_param('/rb_pid')['kd']

MAX_SPEED = 400#rospy.get_param('/rb_max_speed')
motor_speed = 0
lb_speed = 0

######################
address = 12
sn = rospy.get_param('/rb_serial_number')
p = list(serial.tools.list_ports.grep(sn))
port = '/dev/' + p[0].name
######################


target = 0     # target motor position
position = 0   # current motor position

tolerance = 1.0#rospy.get_param('/angle_tolerance')

# Errors for pid controller
current_error = 0
prev_error = 0
accu_error = 0
integral_reset = 75#rospy.get_param('/rb_pid')['ir']

orienbus = orienbus.OrienBus(port)
motor = orienbus.initialize(address)


### Subscriber Functions ###
######################################
# Reading actual encoder position
def encoder_pos(data):
    global position, target
    target = data.linear.x
    position = data.linear.y # take note using correct Twist msg data
    #time = rospy.get_rostime()
    #current_time = time.secs + time.nsecs*(10**(-9))

# Reading target position
def desired_pos(data):
    global lb_speed
    if abs(data.data) < 80:
        lb_speed = 0
    else:
        lb_speed = data.data * 2
     #take note using correct Twist msg data
######################################

### PID Controller Functions ###
######################################
def proportional(desired, actual): #error - current angle
    prop =  kp * (desired - actual)
    #print("Kp: " + str(prop))
    return prop

def derivative(curr, prev, dt): #d angle-error/ dt
    #dt = 0.05 #current_time - prev_time
    #print("dt is: " + str(dt))
    if dt == 0:
        deriv = 0
    else:
        deriv = kd * (curr - prev) / dt
    #print("Kd: "+ str(deriv))
    return deriv

def integral(accu, dt):
    integral =  ki * accu * dt
    #print("Ki: " +str(integral))
    return integral

def control_pid(error, s):
    ks = 1.1  
    ka = 0.5
    output = ks * s * math.tanh(ka * abs(error) + 1)
    return output
######################################

# Adjust motor speed
def adjust_speed(dt):
    global motor_speed
    global current_error
    global prev_error
    global accu_error
    global prev_time
    global integral_reset
    current_error = target - position
    #print("current error is:" + str(current_error))
    accu_error += current_error
    p = proportional(target, position)
    d = derivative(current_error, prev_error, dt)
    i = integral(accu_error, dt)
    '''
    if i > integral_reset:
        i = integral_reset
    if i < -integral_reset:
        i = -integral_reset
    '''
    speed = p + i + d
    speed =  -int(speed) + lb_speed
    #print("input speed is: " + str(speed))
    if abs(current_error) >= tolerance:
        if abs(speed) < abs(MAX_SPEED):
            motor_speed = speed
            motor.writeSpeed(speed)
            #print("Speed: "+str(speed))
        else:
            if speed < 0:
                motor_speed = -MAX_SPEED
                motor.writeSpeed(-MAX_SPEED)
                #print("Speed: "+str(-MAX_SPEED))
            else:
                motor_speed = MAX_SPEED
                motor.writeSpeed(MAX_SPEED)
                #print("Speed: "+str(MAX_SPEED))
    else:
        motor_speed = 0
        motor.writeSpeed(0)
    prev_error = current_error
    #prev_time = current_time
    #print("accu_error: " + str(accu_error))
    
    if abs(i) >= integral_reset:
    	accu_error = 0
    

def callback(req):
    x = False
    if abs(current_error) <= tolerance and abs(motor_speed) < 80:
        x = True
    else:
        x = False
    #print(x)
    return StatusResponse(x)

def kill_procedure():
    print("SHUTTING DOWN NODE")
    motor.writeSpeed(0)
    print("Supposedly off motor")

if __name__ == "__main__":
    try:
        rospy.init_node("rb_circle_steer") # Note correct node name
        rospy.Subscriber("encoder_positions", Twist, encoder_pos)
        rospy.Subscriber("lb_steer_speed", Int64, desired_pos)
        #rospy.Service('/rb_steer_status', Status, callback)
        #rospy.on_shutdown(kill_procedure)
        period = 0.05
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
        #while True:
            if abs(position) >= 95:
                motor.writeSpeed(0)
                break
            else:
                start = rospy.get_time()
                adjust_speed(period)
                #print('adjusting speed')
                rate.sleep()
                end = rospy.get_time()
                period = end - start
            if rospy.is_shutdown() == True:
                motor.writeSpeed(0)
                break
            #print(rospy.is_shutdown())
        motor.writeSpeed(0)
    except rospy.ROSInterruptException:
        motor.writeSpeed(0)
        print("stopping motor")
