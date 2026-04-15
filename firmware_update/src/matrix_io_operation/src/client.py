#! /usr/bin/env python

import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg
from matrix_msgs.msg import *

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('matrix_io_operation', matrix_msgs.msg.IOOperationAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = matrix_msgs.msg.IOOperationGoal()
    ind_goal = PinStateDigital() 
    ind_goal1 = PinStateDigital()
    ind_goal2 = PinStateDigital()
    ind_goal3 = PinStateDigital()

    # ind_goal0 = BoxPose() 
    # goal.order.append(ind_goal0)

    # command = input("enter command:")

    goal.fun = "FUN_SET_OUTPUT"
 
    # goal.project_speed = 30 

    ind_goal.pin = 4
    ind_goal.state = 0
    goal.pin_state.append(ind_goal)

    ind_goal1.pin = 2
    ind_goal1.state = 1
    goal.pin_state.append(ind_goal1)

    ind_goal2.pin = 1
    ind_goal2.state = 0
    goal.pin_state.append(ind_goal2)

    ind_goal3.pin = 4
    ind_goal3.state = 0
    goal.pin_state.append(ind_goal3)

    # ind_goal1.operation_mode = 1
    # ind_goal1.des1 = "machine"
    # ind_goal1.des1_no = 2
    # ind_goal1.des2 = "robot"
    # ind_goal1.des2_no = 2
    # goal.orders.append(ind_goal1)

    # ind_goal2.operation_mode = 2
    # ind_goal2.des1 = "robot"
    # ind_goal2.des1_no = 3
    # ind_goal2.des2 = "machine"
    # ind_goal2.des2_no = 3
    # goal.orders.append(ind_goal2)

    # ind_goal3.operation_mode = 2
    # ind_goal3.des1 = "robot"
    # ind_goal3.des1_no = 4
    # ind_goal3.des2 = "machine"
    # ind_goal3.des2_no = 4
    # goal.orders.append(ind_goal3)

    # ind_goal2.operation_mode = 1
    # ind_goal2.des1 = "robot"
    # ind_goal2.des1_no = 3
    # ind_goal2.des2 = "machine"
    # ind_goal2.des2_no = 3
    # goal.orders.append(ind_goal2)

    # ind_goal3.operation_mode = 1
    # ind_goal3.des1 = "robot"
    # ind_goal3.des1_no = 4
    # ind_goal3.des2 = "machine"
    # ind_goal3.des2_no = 4
    # goal.orders.append(ind_goal3)



    #ind_goal1.pose = 1
    #ind_goal1.command = int(command)
    # goal.order.append(ind_goal1)

    # ind_goal2.pose = 2
    # ind_goal2.command = 177
    # goal.order.append(ind_goal2)

    # ind_goal3.pose = 1
    # ind_goal3.command = 177
    # goal.order.append(ind_goal3)
    #print(goal)

    # ind_goal2.pose = 4
    # ind_goal2.command = 177
    # goal.order.append(ind_goal2)
    # print(goal)


    # ind_goal3.pose = 2
    # ind_goal3.command = 177
    # goal.order.append(ind_goal3)
    # print(goal)
    
    # ind_goal.pose = 5
    # ind_goal.command = 1
    # goal.order.append(ind_goal)

    # print(goal)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        pass
        # print("program interrupted before completion", file=sys.stderr)