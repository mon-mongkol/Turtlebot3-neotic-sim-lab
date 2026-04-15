#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt8
import usb.core
from aragorn_bringup.msg import DiagPort

wheel_right =0 
wheel_left =0
io =0
imu =0
bms = 0
lidar = 0

def checkusb():

    pub = rospy.Publisher('usb_diagnostics', DiagPort, queue_size=2)
    rospy.init_node('checkusb', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
                                                                                  #usb_int = 0 : diagnostics is Ready !!
        aragorn_wheel_right = usb.core.find(idVendor=0x403, idProduct=0x6015)     #usb_int = 1 : aragorn_wheel_right  USB  not found  !!
        aragorn_wheel_left  = usb.core.find(idVendor=0x0557, idProduct=0x2008)    #usb_int = 2 : aragorn_wheel_left  USB  not found  !!
        aragorn_io    = usb.core.find(idVendor=0x16c0, idProduct=0x0483)            #usb_int = 3 : aragorn_io  USB  not found  !!
        aragorn_lidar = usb.core.find(idVendor=0x10c4, idProduct=0xea60)          #usb_int = 4 : aragorn_lidar  USB  not found  !!
        aragorn_imu = usb.core.find(idVendor=0x0483, idProduct=0x5740)            #usb_int = 5 : aragorn_imu  USB  not found  !!
        aragorn_bms = usb.core.find(idVendor=0x1a86, idProduct=0x7523)            #usb_int = 6 : aragorn_bms  USB  not found  !!
        
       

        if aragorn_wheel_right is None :
            usb_int = 1 
            rospy.logerr("aragorn_wheel_right_USB not found")
            pub.publish(usb_int)
            rate.sleep()
            wheel_right = 0
            # raise ValueError('"aragorn_wheel_right not found')   #For stop run
        else:
            wheel_right = 1

        if aragorn_wheel_left is None :
            usb_int = 2 
            rospy.logerr("aragorn_wheel_left_USB not found")
            pub.publish(usb_int)
            rate.sleep()
            wheel_left = 0
            # raise ValueError('"aragorn_wheel_left not found')
        else:
            wheel_left = 1

        if aragorn_io is None :
            usb_int = 3 
            rospy.logerr("aragorn_io_USB not found")
            pub.publish(usb_int)
            rate.sleep()
            io = 0
            # raise ValueError('"aragorn_io not found')
        else:
            io = 1

        
        if aragorn_lidar is None :
            usb_int = 4
            rospy.logerr("aragorn_lidar_USB not found")
            pub.publish(usb_int)
            rate.sleep()
            lidar = 0
            # raise ValueError('"aragorn_lidar not found')
        else:
            lidar = 1

        # if aragorn_imu is None :
        #     usb_int = 5
        #     rospy.logerr("aragorn_imu_USB not found")
        #     pub.publish(usb_int)
        #     rate.sleep()
        #     imu = 0
        #     # raise ValueError('"aragorn_imu not found')
        # else:
        #     imu = 1

        # if aragorn_bms is None :
        #     usb_int = 6
        #     rospy.logerr("aragorn_bms_USB not found")
        #     pub.publish(usb_int)
        #     rate.sleep()
        #     bms = 0
        #     # raise ValueError('"aragorn_bms not found')
        # else:
        #     bms = 1

        sum_data = wheel_right + wheel_left + io + lidar  #imu + bms
        if sum_data == 4:   #6:
            usb_int = 0
            rospy.loginfo("aragorn_ready !!!")
            pub.publish(usb_int)
            rate.sleep()
        else:
            None

if __name__ == '__main__':
    try:
        checkusb()
    except rospy.ROSInterruptException:
        pass

