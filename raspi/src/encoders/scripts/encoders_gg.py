#!/usr/bin/env python

import rospy
from encoders.srv import *
from geometry_msgs.msg import Twist
import arduino

wheel_track = 0.156
meter_per_tick = 0.0001309

def server_srv():
    rospy.init_node("encoders")
    s = rospy.Service("encoders", encoders, handle_function)
    rospy.loginfo("Ready to return the encoders' counts:")
    rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, send_velocity)
    rospy.spin()

# Define the handle function to handle the request inputs
def handle_function(req):
    if req.request == True:
        counts = myArduino.get_encoder_counts()
    return encodersResponse(str(counts[0])+'|'+str(counts[1])+'$')

def send_velocity(Twist):
    v_center = Twist.linear.x
    v_th = Twist.angular.z

    v_left = v_center - v_th * wheel_track / 2.0
    v_right = v_center + v_th * wheel_track / 2.0

    if v_left * v_right >= 0:
        #if v_left < 0 or v_right < 0:
        #    myArduino.motors_reversed = True
        #else:
        #    myArduino.motors_reversed = False
        print (v_left / meter_per_tick) * 2.0
        
        myArduino.drive((v_left / meter_per_tick) * 2.0 , (v_right / meter_per_tick) * 2.0)

if __name__=="__main__":
    portName = "/dev/ttyACM0"
    baudRate = 57600

    myArduino = arduino.Arduino(port=portName, baudrate=baudRate, timeout=0.5)
    myArduino.connect()

    server_srv()
