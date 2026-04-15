#!/usr/bin/env python3
import rospy
from std_msgs.msg import *
from std_srvs.srv import SetBool
from sensor_msgs.msg import *
import time
from dynamic_reconfigure.server import Server
from matrix_amr_bringup.cfg import VirtualIOConfig
from matrix_msgs.msg import *
from lino_msgs.msg import *

class NumberCounter:
    def __init__(self):

        self.pub_bumper = rospy.Publisher("/matrix_io/bumper", Int32MultiArray, queue_size=1)
        self.pub_emergency = rospy.Publisher("/matrix_io/emergency", Int8, queue_size=1)
        self.pub_heart_beat = rospy.Publisher("/matrix_io/heart_beat", Int8, queue_size=1)
        self.pub_input = rospy.Publisher("/matrix_io/input", Int32MultiArray, queue_size=1)
        self.pub_ios_state = rospy.Publisher("/matrix_io/ios_state", IOs_state, queue_size=1)
        self.pub_maxonar_raw = rospy.Publisher("/matrix_io/maxsonar_raw", Int32MultiArray, queue_size=1)
        self.pub_output = rospy.Publisher("/matrix_io/output", Int32MultiArray, queue_size=1)
        self.pub_raw_imu = rospy.Publisher("/matrix_io/raw_imu", Imu, queue_size=1)
        self.pub_rs485_receive = rospy.Publisher("/matrix_io/rs485_receive", String, queue_size=1)
        self.pub_software_info = rospy.Publisher("/matrix_io/software_info", String, queue_size=1)
        self.pub_ultrasonic_raw = rospy.Publisher("/matrix_io/ultrasonic_raw", Int32MultiArray, queue_size=1)
        self.pub_soft_start_imu = rospy.Subscriber("/matrix_io/soft_start_imu", Bool, queue_size=1)

        self.sub_robot_status = rospy.Subscriber("/matrix_system/movement_status", MovmentStatus, self.cbRobotStatus)
        self.sub_robot_mode = rospy.Subscriber("/matrix_mode_controller/mode", RobotMode, self.cbRobotMode)
        self.sub_rs485_send = rospy.Subscriber("/matrix_io/rs485_send", String, self.cbRS485Send)
        self.sub_user_ctrl_output = rospy.Subscriber("/matrix_io_management/user_ctrl_output", Int32MultiArray, self.cbUserControlOutput)
        self.sub_allowed_master_on = rospy.Subscriber("/matrix_mode_controller/allowed_master_on", Bool, self.cbAllowedMO)
        self.sub_busbar = rospy.Subscriber("/matrix_io/busbar", Bool, self.cbBusbar)


        self.srv = Server(VirtualIOConfig, self.callback)



        self.XBumper = 1
        self.XEmergency = 1
        self.XEmergency_plugin = 1
        self.XMaster_on = 1
        self.XMaster_on_done = 1
        self.XEmergency_charge = 1
        self.XLimit_switch = 1
        self.XCustom_1 = 1
        self.XCustom_2 = 1
        self.XCustom_3 = 1
        self.XCustom_4 = 1

        self.YComputer_ready = 0
        self.YShutdown = 0
        self.YCutoff = 0
        self.YEnable_busbar = 0
        self.YOut_1 = 2
        self.YOut_2 = 2
        self.YOut_3 = 2
        self.YOut_4 = 2
        self.YRelay_start_imu = 1


        self._AllowMasterOn = False
        self.USE_MASTER_ON_SYSTEM = 1
        self._isInit_state = False


        self.prev_heartbeat_time = 0
        self.prev_emergency_time = 0
        self.prev_led_control_time = 0
        self.prev_master_on_check_time = 0
        self.prev_pub_ultrasonic_time = 0
        self.prev_pub_serial485 = 0
        self.disconnect_master_stmp = 0
        self.prev_pub_softstart_imu = 0
        self.prev_imu_time = 0

        
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.main_control()
            rate.sleep()
    
    def cbRobotStatus(self, msg):
        pass

    def cbRobotMode(self, msg):
        pass
    
    def cbRS485Send(self, msg):
        pass

    def cbUserControlOutput(self, msg):
        pass

    def cbAllowedMO(self, msg):
        self._AllowMasterOn = msg.data

    def cbBusbar(self, msg):
        if msg.data:
            self.YEnable_busbar = 1
        else:
            self.YEnable_busbar = 0
    
    def convert_state(self, bool_state):
        state = 0 if bool_state == True else 1
        return state
    
    def callback(self, config, level):
        self.XBumper = self.convert_state(config.bumper_4)
        self.XEmergency = self.convert_state(config.emergency_0)
        self.XEmergency_plugin = self.convert_state(config.emergency_0)
        self.XMaster_on = self.convert_state(config.master_on_1)
        self.XEmergency_charge = self.convert_state(config.emergency_charge_3)
        self.XLimit_switch = self.convert_state(config.limit_switch_7)

        self.XCustom_1 = self.convert_state(config.custom_in_1_10)
        self.XCustom_2 = self.convert_state(config.custom_in_2_11)
        self.XCustom_3 = self.convert_state(config.custom_in_3_12)
        self.XCustom_4 = self.convert_state(config.custom_in_4_13)

        config.master_on_1 = False
        config.custom_in_1_10 = False
        config.custom_in_2_11 = False
        config.custom_in_3_12 = False
        config.custom_in_4_13 = False

        return config

    def main_control(self):
        self.button_control()
    
    def pub_bumper(self):
        # self.getBumper()
        msg = Int32MultiArray()
        if self.XBumper == 0:
            msg.data = [0, 0]
        else:
            msg.data = [1, 1]
        self.pub_bumper.publish()

    
    def SoftStart_IMU(self):
        msg = Bool()
        if self.YRelay_start_imu == 1:
            msg.data = True
        else:
            self.YRelay_start_imu = 0
            msg.data = False
        self.pub_soft_start_imu.publish(msg)
    
    def pub_hb(self):
        msg = Int8
        msg.data = 1
        self.pub_heart_beat.publish(msg)
    
    def pub_emergency(self):
        msg = Int8
        if (self.XEmergency == 0) or (self.XEmergency_plugin == 0):
            msg.data = 0
        else:
            msg.data = 1
        self.pub_emergency.publish(msg)

    
    def emergency_mode(self):
        self.init_state()
    
    def init_state(self):
        self.YComputer_ready = 0
        self.YEnable_busbar = 0
    
    def systemDiagnostics(self):
        if not self._AllowMasterOn:
            self.emergency_mode()
    
    def button_control(self):
        if self.XEmergency == 0:
            self.emergency_mode()
            if self.USE_MASTER_ON_SYSTEM == 1:
                if not self._AllowMasterOn:
                    if self.XMaster_on == 0:
                        rospy.loginfo("[Matrix_io]: MCU state didn't Allow to master on, please check emergency and robot_state139")
                
                self._isInit_state = False
        else:
            if self.USE_MASTER_ON_SYSTEM == 1:
                if (not self._isInit_state) and (self._AllowMasterOn):
                    rospy.loginfo("[Matrix_io]: Robot ready for master on!!!")
                    # self.init_state()
                    self._isInit_state = True
                else:
                    pass

                if self._AllowMasterOn:
                    if self.XMaster_on_done != 0:
                        if self.XMaster_on == 0:
                            self.YComputer_ready = 1
                else:
                    rospy.loginfo("[Matrix_io]: MCU state didn't Allow to master on, please check emergency and robot_state 213")
                    self._isInit_state = True

if __name__ == '__main__':
    rospy.init_node('number_counter')
    NumberCounter()
    rospy.spin()