#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from husky_modeswitch.msg import *
import serial

def ptu_callback(data):
#Eliminate small offsets of the analog stick if it is not exactly centered
    global Pan_value, Tilt_value, Pan_value_to_send, Tilt_value_to_send
    #if ((data.tilt_angle>0.08) or (data.tilt_angle<-0.08)): #or ((data.pan_angle>0.08) and (data.pan_angle<-0.08)):
#Values coming will be ranged from [-1,1]; So scale these as needed. Here it is 5 times scale.
    Pan_value = Pan_value + (5*data.tilt_angle)
    Tilt_value = Tilt_value - (5*data.pan_angle)
    Pan_value_to_send = "pp%d " %Pan_value
    print Pan_value_to_send
    Tilt_value_to_send = "tp%d " %Tilt_value
    PTU.write(Pan_value_to_send)
    PTU.write(Tilt_value_to_send)

if __name__ == '__main__':

    global PTU
    global Pan_value, Tilt_value, Pan_value_to_send, Tilt_value_to_send
    Pan_value = 0
    Tilt_value = 0
    Tilt_value_to_send = 0
    Pan_value_to_send = 0

    PTU = serial.Serial(
        port ='/dev/ttyUSB0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    rospy.init_node('husky_ptu', anonymous=True)
    rospy.Subscriber('/husky_modeswitch/controlhardware_interface',husky_controlinfo,ptu_callback)
    rospy.spin()
