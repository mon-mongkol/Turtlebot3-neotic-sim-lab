#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from can_msgs.msg import *
from struct import *
import numpy as np

def callback(data):
   message = data
#    a = np.frombuffer(data.data, dtype=np.uint8)
#    if a[0] == 0x43 and a[1] == 0x6c and a[2] == 0x60 and a[3] == 0x00:
#        print(hex(a[0]))
#    print(a)
   if message.data[0] == 0x43 and message.data[1] == 0x6c and message.data[2] == 0x60 and message.data[3] == 0x00:
       print("aaaa")

    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/received_messages", Frame, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()