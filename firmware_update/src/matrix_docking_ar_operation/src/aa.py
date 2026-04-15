#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64, String
from std_srvs.srv import SetBool
from matrix_msgs.msg import *

class NumberCounter:

    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)
        self.sub_rs485 = rospy.Subscriber("/matrix_io/rs485_send", String, self.callback_rs485_send)
        self.pub_rs485 = rospy.Publisher("/matrix_io/rs485_receive", String, queue_size=1) 
    
    def callback_rs485_send(self, msg):
        msg_back = String()
        if msg.data == "#CHARGE_FINISH4$":
            msg_back.data = "CG_READY4"
        elif msg.data == "#ROBOT_WAIT_ROBOT_REQ_CG4$":
            msg_back.data = "WAIT_ROBOT_REQ_CG4"
        elif msg.data == "#CHARGE_REQ4$":
            msg_back.data = "CG_REQ_SUCCESS4"
        elif msg.data == "#ROBOT_IN_CHARGING_STATE4$":
            msg_back.data = "CG_CHARGING_STATE4"
        else:
            pass
        # rospy.sleep(2)
        self.pub_rs485.publish(msg_back)
        



    def callback_number(self, msg):
        self.counter += msg.data
        new_msg = Int64()
        new_msg.data = self.counter
        self.pub.publish(new_msg)

    def callback_reset_counter(self, req):
        if req.data:
            self.counter = 0
            return True, "Counter has been successfully reset"
        return False, "Counter has not been reset"

if __name__ == '__main__':
    rospy.init_node('number_counter')
    NumberCounter()
    rospy.spin()r