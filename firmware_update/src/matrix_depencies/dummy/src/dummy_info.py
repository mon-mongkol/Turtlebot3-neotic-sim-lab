#!/usr/bin/env python3
# license removed for brevity
import rospy
import random
from std_msgs.msg import Int64



def talker():
    pub_robot_mode = rospy.Publisher('/robot_mode', Int64, queue_size=10)
    pob_building = rospy.Publisher('/building', Int64, queue_size=10)
    pub_level = rospy.Publisher('/level', Int64, queue_size=10)
    pub_poi= rospy.Publisher('/poi', Int64, queue_size=10)

    rospy.init_node('MODE', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        mode = Int64()
        building = Int64()
        level= Int64()
        poi = Int64()

        building.data =10  #random.randint(1, 99)
        level.data = 2 #random.randint(1, 99)
        poi.data = 6 #random.randint(1, 999)
        mode.data = random.randint(13, 14)

        pob_building.publish(building)
        pub_level.publish(level)
        pub_poi.publish(poi)
        # pub_robot_mode.publish(mode)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
