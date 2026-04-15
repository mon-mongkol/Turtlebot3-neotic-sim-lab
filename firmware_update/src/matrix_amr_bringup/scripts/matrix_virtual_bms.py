#!/usr/bin/env python3
import rospy
from std_msgs.msg import *
from std_srvs.srv import SetBool
from sensor_msgs.msg import *
import time
from dynamic_reconfigure.server import Server
from matrix_amr_bringup.cfg import VirtualBatteryConfig

class NumberCounter:
    def __init__(self):
        self.counter = 0
        # self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)

        self.batt_pub = rospy.Publisher("/battery_state", BatteryState, queue_size=1)
        self.temp_pub = rospy.Publisher("/battery_temperature", Temperature, queue_size=1)
        self.charging_cycle = rospy.Publisher("/battery_charge_cycles", Int16, queue_size=1)

        # self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        # self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)

        self.voltage_cfg = 26.1
        self.current_cfg = -3.4
        self.current_charge_cfg = 0
        self.power_supply_status_cfg = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.power_supply_health_cfg = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.power_supply_technology_cfg = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        self.cap_cfg = 48
        self.cell_voltage = [3.148, 3.2548, 3.187, 3.145, 3.119, 3.167, 3.121, 3.190]
        self.percentage = 98.00

        self.prev_cal_time = 0 
    
        
        self.batter_mode = 6
        
        self.battery_serial_no = "SMR020020230001ARB030SIM"
        try:
            self.battery_serial_no = rospy.get_param('~battery_serial_no', 'SMR020020230001ARB030SIM')
            self.batter_cfg_state = rospy.get_param('~batter_cfg_state', 6)
        except:
            print("Read Battery serial error")


        self.srv = Server(VirtualBatteryConfig, self.callback)

        
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.main_pub()
            rate.sleep()
    
    def callback(self, config, level):
        # rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\ 
        #     {str_param}, {bool_param}, {size}""".format(**config))
        self.batter_mode = config.batter_cfg_state
        self.battery_serial_no = config.battery_serial_no
        # print(self.batter_mode)
        return config

    def cal(self):

        
        if self.batter_mode == 2: #Discharge from dynamics reconfigure
            # self.current_cfg = self.current_cfg if self.current_cfg < -1 else self.current_cfg*-1
            self.current_cfg = -3.4
            self.current_charge_cfg = 0
            self.power_supply_status_cfg = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            
            # self.voltage_cfg = self.voltage_cfg
        else:
            self.current_charge_cfg = 15
            self.current_cfg = self.current_charge_cfg -3.4 
            self.power_supply_status_cfg = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            
            

            
        if ((rospy.Time.from_sec(time.time()).to_sec() - self.prev_cal_time) >= 60):        
            if self.batter_mode == 2:
                self.percentage -= 0.1
            else:
                if self.percentage < 100:
                    self.percentage += 0.1
            self.prev_cal_time = rospy.Time.from_sec(time.time()).to_sec()
        else:
            pass

        self.percentage = 0 if self.percentage < 0 else 100 if self.percentage > 100 else self.percentage
        
        # if self.batter_mode == 2: #Discharge from dynamics reconfigure
        #     self.current_cfg = self.current_cfg if self.current_cfg < -1 else self.current_cfg*-1
        #     self.current_charge_cfg = 0
        #     self.power_supply_status_cfg = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        #     # self.voltage_cfg = self.voltage_cfg
        # else:
        #     self.current_charge_cfg = 15
        #     self.current_cfg = self.current_charge_cfg - abs(self.current_cfg) 
        #     self.power_supply_status_cfg = BatteryState.POWER_SUPPLY_STATUS_CHARGING



    
    def main_pub(self):
        self.cal()
        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.voltage = self.voltage_cfg
        msg.current = self.current_cfg
        msg.capacity = self.cap_cfg
        msg.charge = self.current_charge_cfg
        msg.design_capacity = 50
        msg.percentage = self.percentage
        msg.present = True
        msg.cell_voltage = self.cell_voltage
        msg.power_supply_health = self.power_supply_health_cfg
        msg.power_supply_status = self.power_supply_status_cfg 
        msg.power_supply_technology = self.power_supply_technology_cfg
        msg.location = "battery_pack_link"
        msg.serial_number = self.battery_serial_no        
        self.batt_pub.publish(msg)

        msg_temp = Temperature()
        msg_temp.header.stamp = rospy.Time.now()
        msg_temp.temperature = 31.18
        self.temp_pub.publish(msg_temp)

        msg_cycle = Int16()
        msg_cycle.data = 7
        self.charging_cycle.publish(msg_cycle)

        
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
    rospy.spin()