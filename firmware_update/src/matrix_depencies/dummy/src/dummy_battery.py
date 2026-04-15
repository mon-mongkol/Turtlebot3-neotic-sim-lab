#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import BatteryState
import random

def talker():
    pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        rospy.set_param('/matrix_diagnostics/serial_no','ROHM020020220001A')
        hello_str = BatteryState()
        hello_str.design_capacity = 150.0
        hello_str.percentage = random.randint(5, 10)*0.1
        hello_str.capacity = (hello_str.design_capacity)*(hello_str.percentage)
        hello_str.power_supply_status = 2
        hello_str.power_supply_health = 0
        hello_str.power_supply_technology = 4
        hello_str.cell_voltage = [random.randint(325, 330)/100.0 for i in range(8)]
        hello_str.present = True
        hello_str.voltage = sum(hello_str.cell_voltage)
        hello_str.current =-2.7
        hello_str.current = (random.randint(220, 300)/100.0)*-1
        hello_str.serial_number = 'ROHM020020220001A'
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass