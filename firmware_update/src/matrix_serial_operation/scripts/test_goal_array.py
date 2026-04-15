#! /usr/bin/env python3

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
    client = actionlib.SimpleActionClient('matrix_serial_operation_485', matrix_msgs.msg.SerialOperationAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = matrix_msgs.msg.SerialOperationGoal()
    ind_goal = "a"
    ind_goal1 = "freex"
    ind_goal2 = "c"
    ind_goal3 = "d"
    ind_goal4 = "e3ee"
    ind_goal5 = "RxID"
    # ind_goal6 = "g"
    # ind_goal7 = "h"
    # ind_goal8 = "i"
    # ind_goal9 = "j"
    # ind_goal10 = "k"
    # ind_goal11 = "l"
    # ind_goal12 = "m"
    # ind_goal13 = "n"
    # ind_goal14 = "o"
    # ind_goal15 = "p"
    # ind_goal16 = "q"
    # ind_goal17 = "r"
    # ind_goal18 = "w"
    # ind_goal19 = "t"

    # command = input("enter command:")

    # goal.mode = 1
    
    goal.cmd = "find_data_in_payload"
    goal.timeout = 100 #timeout
    goal.arg_sr_send ="go"
    goal.arg_sr_send_repeats=True
    goal.arg_int = 0
    goal.arg_strings.append(ind_goal)
    goal.arg_strings.append(ind_goal1)
    goal.arg_strings.append(ind_goal2)
    goal.arg_strings.append(ind_goal3)
    goal.arg_strings.append(ind_goal4)
    goal.arg_strings.append(ind_goal5)
    # goal.arg_strings.append(ind_goal6)
    # goal.arg_strings.append(ind_goal7)
    # goal.arg_strings.append(ind_goal8)
    # goal.arg_strings.append(ind_goal9)
    # goal.arg_strings.append(ind_goal10)
    # goal.arg_strings.append(ind_goal11)
    # goal.arg_strings.append(ind_goal12)
    # goal.arg_strings.append(ind_goal13)
    # goal.arg_strings.append(ind_goal14)
    # goal.arg_strings.append(ind_goal15)
    # goal.arg_strings.append(ind_goal16)
    # goal.arg_strings.append(ind_goal17)
    # goal.arg_strings.append(ind_goal18)
    # goal.arg_strings.append(ind_goal19)



    # ind_goal.pose = 2
    # ind_goal.command = int(command)
    # goal.order.append(ind_goal)

    # ind_goal1.pose = 1
    # ind_goal1.command = int(command)
    # goal.order.append(ind_goal1)

    # ind_goal2.pose = 2
    # ind_goal2.command = 177
    # goal.order.append(ind_goal2)

    # ind_goal3.pose = 1
    # ind_goal3.command = 177
    # goal.order.append(ind_goal3)
    print(goal)

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