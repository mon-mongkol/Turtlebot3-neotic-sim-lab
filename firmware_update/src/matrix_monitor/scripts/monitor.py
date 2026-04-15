#!/usr/bin/env python3

import rospy
import sys
import base64
import time

import os
import json
from struct import *
from std_msgs.msg import String ,Float32
from geometry_msgs.msg import Pose
from matrix_msgs.msg import RobotMode , MovmentStatus , Diagnostics_modern
from sensor_msgs.msg import BatteryState
from web_interface_msgs.msg import IntersectPOI
from datetime import datetime
from matrix_monitor.monitorAPIWS import monitorAPIWS

from flask import Flask
from flask_mysqldb import MySQL
from flask_restful import Resource, Api, reqparse
# from flask_cors import CORS

app = Flask(__name__)
api = Api(app)
# CORS(app)

hostname = 'localhost'
db_name = 'istuvd'
db_user = 'istdbUser'
db_password = 'interface2563'

app.config['MYSQL_HOST'] = hostname
app.config['MYSQL_USER'] = db_user
app.config['MYSQL_PASSWORD'] = db_password
app.config['MYSQL_DB'] = db_name

mysql = MySQL(app)
parser = reqparse.RequestParser()


class Monitor:

    def __init__(self):

        self.socket_io_monitor = False
        self.socket_io_monitor_ip = ""
        self.socket_io_monitor_port = ""
        try:
            with app.app_context():
                try:
                    conn = mysql.connection
                    cur = conn.cursor()
                    cur.execute(
                        "SELECT configs  FROM `ros_options` where name = 'MONITOR'")
                    response = cur.fetchone()
                    # rospy.loginfo("Read form db:\n configs : %s",
                    #             response[0])
                    resp = json.loads(response[0])
                    if(resp["protocal"] == "Ethernet"):
                        self.socket_io_monitor = True
                    #true /false
                    self.socket_io_monitor_ip = resp["host"]
                    #192.168.12.141
                    self.socket_io_monitor_port = resp["port"]
                    #5000
                    mysql.connection.commit()
                    cur.close()
                except Exception as e:
                    rospy.loginfo(e)
        except:
            pass

        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0
        self.robot_mode = 0
        self.batt_percent = 0.0
        self.serial = ""
        self.map_info = ""
        self.map_name = ""
        self.robot_on_poi = ""
        self.moving = 0
        self.odo = 0
        self.data_diagnostic = []
        self.device_error = []
        self.ini_time_for_now = datetime.now()
        
        if(self.socket_io_monitor):
            rospy.loginfo("Connect to WS.: "+"http://"+self.socket_io_monitor_ip+":"+self.socket_io_monitor_port)
            self.monitorAPIWS = monitorAPIWS()
            self.monitorAPIWS.set_ip("http://"+self.socket_io_monitor_ip+":"+self.socket_io_monitor_port)
            while(not self.monitorAPIWS.connected):
                
                self.monitorAPIWS.run()
                print(self.monitorAPIWS.connected)
                time.sleep(2)

        # Initialize status
        rospy.loginfo('Initializing Robot Monitor.')
        ### ============================================###
        ###                   PUBLISH                   ###
        ### ============================================###
        self.monitor_pub = rospy.Publisher(
            'remote_xbee',
            String ,queue_size=10
            )
        
        ### ============================================###
        ###                   Subscribe                 ###
        ### ============================================###
        self.current_robotpose = rospy.Subscriber(
            'robot_pose',
            Pose,
            self.current_robotpose_callback)
        self.robotpoi = rospy.Subscriber(
            'robot_on_poi',
            IntersectPOI,
            self.robot_on_poi_callback)
        self.robotmode = rospy.Subscriber(
            'matrix_mode_controller/mode' ,
            RobotMode ,
            self.robotmode_callback)
        self.batterystate = rospy.Subscriber(
            'battery_state',
            BatteryState,
            self.battery_state_callback)
        self.mapinfo = rospy.Subscriber(
            'map_info',
            String,
            self.map_info_callback)
        self.movement = rospy.Subscriber(
            'matrix_system/movement_status',
            MovmentStatus,
            self.movemento_callback)
        self.odometer = rospy.Subscriber(
            'odo',
            Float32,
            self.odometer_callback)
        self.diagnostics = rospy.Subscriber(
            'matrix_system/diagnostics',
            Diagnostics_modern,
            self.diagnostics_callback)

    def publish_state(self,data):
        data = str(data)
        self.monitor_pub.publish(data)

    def odometer_callback(self,msg):
        if (self.odo != msg.data):
            self.odo = msg.data

    def current_robotpose_callback(self, msg):
        if( (abs(abs(self.position_x) - abs(msg.position.x)) > 0.02) or
        (abs(abs(self.position_y) - abs(msg.position.y)) > 0.02) or
        (abs(abs(self.orientation_z) - abs(msg.orientation.z)) > 0.02) or
        (abs(abs(self.orientation_w) - abs(msg.orientation.w)) >0.02)) :
            self.position_x = msg.position.x
            self.position_y = msg.position.y
            self.orientation_z = msg.orientation.z
            self.orientation_w = msg.orientation.w
            #rospy.loginfo("\nX:%s\nY:%s\nZ:%s\nW:%s",self.position_x,self.position_y,self.orientation_z,self.orientation_w)

    def robot_on_poi_callback(self,msg):
        if (self.robot_on_poi != msg.current_poi):
            self.robot_on_poi = msg.current_poi
            #rospy.loginfo("\n%s",self.robot_on_poi)

    def robotmode_callback(self, msg):
        self.robot_mode = msg.robot_mode
        time = datetime.now()
        diff = time - self.ini_time_for_now
        #print(msg.robot_mode)
        if(round(diff.total_seconds(),1)> 1800 ):
            if (self.robot_mode == 2):
                #print(diff)
                self.robot_mode = 99
                #print(self.robot_mode)
        # if (self.robot_mode != msg.robot_mode):
        if(msg.robot_mode !=2 or self.moving ==1):
            self.robot_mode = msg.robot_mode
            self.ini_time_for_now = time
            #print(diff)
            #rospy.loginfo("\n%s",self.robot_mode)
        
        #print(self.robot_mode)

    def battery_state_callback(self, msg):
        #if (self.batt_percent != msg.percentage):
        self.batt_percent = msg.percentage
        self.serial = msg.serial_number[11:]
        #rospy.loginfo("\n%s\n%s",self.serial,self.batt_percent)

    def map_info_callback(self, msg):
        if (self.map_info != msg.data):
            self.map_info = msg.data
            item = msg.data
            split_data = item.split(" ")
            self.map_name = split_data[0]
            #rospy.loginfo("\n%s",self.map_name)
    def movemento_callback(self , msg):
        if (self.moving != msg.robot_moving):
            self.moving = msg.robot_moving
            #rospy.loginfo("\n%s",self.moving)

    def diagnostics_callback(self , msg):
        if (msg.system_ready==2):
            self.data_diagnostic = msg.status
            self.device_error = []
            for number in range(0,len(self.data_diagnostic),1):
                if(self.data_diagnostic[number].level == 2):
                    self.device_error.append(self.data_diagnostic[number].name)

            # rospy.loginfo("LENGTH DIA\n%s",len(self.data_diagnostic))
            # rospy.loginfo("\nERROR %s",self.device_error)

    def monitor_data(self):
        # rospy.loginfo("\nX:%s\nY:%s\nZ:%s\nW:%s\nPOI:%s\nMode:%s\nSerial:%s\nPercent:%s\nMap:%s\nMoving:%s",    
        #               self.position_x,
        #               self.position_y,
        #               self.orientation_z,
        #               self.orientation_w,
        #               self.robot_on_poi,
        #               self.robot_mode,
        #               self.serial,
        #               self.batt_percent,
        #               self.map_name,
        #               self.moving)
        return_data = '\x02'+self.serial+"&" \
                        +str(round(self.batt_percent,2))+"&" \
                        +self.robot_on_poi+"&" \
                        +str(self.robot_mode)+"&" \
                        +self.map_name+"&" \
                        +str(round(self.position_x,2))+"&" \
                        +str(round(self.position_y,2))+"&" \
                        +str(round(self.orientation_z,2))+"&" \
                        +str(round(self.orientation_w,2))+"&" \
        #                 +str(self.moving)+'\x03'
        
        # data = self.serial+"&" \
        #     +str(round(self.batt_percent,2))+"&" \
        #     +self.robot_on_poi+"&" \
        #     +str(self.robot_mode)+"&" \
        #     +self.map_name+"&" \
        #     +str(round(self.position_x,2))+"&" \
        #     +str(round(self.position_y,2))+"&" \
        #     +str(round(self.orientation_z,2))+"&" \
        #     +str(round(self.orientation_w,2))+"&" \
        #     +str(self.moving)
        # encode_byte = data.encode("ascii")
        # base64_byte = base64.b64encode(encode_byte)
        
        package = pack('>iIiiiiIi',
             int(round(self.batt_percent,2)*100),
             self.robot_mode,
             int(round(self.position_x,2)*100),
             int(round(self.position_y,2)*100),
             int(round(self.orientation_z,2)*100),
             int(round(self.orientation_w,2)*100),
             self.moving,
	         int(round(self.odo,0))
             )
        unpackage = unpack('>iIiiiiIi',package)

        #rospy.loginfo("%s",base64_byte)
        # rospy.loginfo("64:%d",len(base64_byte))
        # rospy.loginfo("RW:%d",len(return_data))
        # rospy.loginfo("PK:%d\n%s",len(package),package)
        # rospy.loginfo("UK:%d\n%s",len(unpackage),unpackage[1])
        # return_data = '\x02'+str(base64_byte)+'\x03'

        data_to_crc = self.serial \
                        +self.robot_on_poi \
                        +self.map_name \
                        +str(unpackage[0]) \
                        +str(unpackage[1]) \
                        +str(unpackage[2]) \
                        +str(unpackage[3]) \
                        +str(unpackage[4]) \
                        +str(unpackage[5]) \
                        +str(unpackage[6]) \
			            +str(unpackage[7])


        new_data = bytearray(len(data_to_crc))
        new_data = data_to_crc.encode('utf-8')
        # print(new_data)
        # print(data_to_crc)
        high , low = self.crcJK232(new_data)
        # print(hex(high) ,hex(low))
        # print(type(hex(high)) ,type(hex(low)))

        
        return_datapack = '\x02'+self.serial+'\x1F' \
                        +self.robot_on_poi+'\x1F' \
                        +self.map_name+'\x1F' \
                        +str(unpackage[0])+'\x1F' \
                        +str(unpackage[1])+'\x1F' \
                        +str(unpackage[2])+'\x1F' \
                        +str(unpackage[3])+'\x1F' \
                        +str(unpackage[4])+'\x1F' \
                        +str(unpackage[5])+'\x1F' \
                        +str(unpackage[6])+'\x1F' \
			            +str(unpackage[7])+'\x1F' \
                        +str(high)+'\x1F'\
                        +str(low) \
                        +'\x03'

        if (self.robot_mode == 14):
            edata_to_crc = self.serial
            device_error_data = ""
            return_datapack = '\x02'+self.serial
            for number in range(0,len(self.device_error),1):
                edata_to_crc += self.device_error[number]
                device_error_data += self.device_error[number]
                return_datapack += '\x11'+self.device_error[number]
            
            e_new_data = bytearray(len(edata_to_crc))
            e_new_data = edata_to_crc.encode('utf-8')
            # print(new_data)
            # print(data_to_crc)
            e_high , e_low = self.crcJK232(e_new_data)
                            
            return_datapack += '\x11'+str(e_high)+'\x11'+str(e_low)+'\x03'
            
            if(self.socket_io_monitor):
                self.monitorAPIWS.postrobottomonitorerror(self.serial,device_error_data)
            else:
                rospy.loginfo("RP:%s",return_datapack)
                self.publish_state(return_datapack)

        if (self.serial != "" and self.robot_mode != 14):
            # rospy.loginfo("RW:%d",len(return_data))
            if(self.socket_io_monitor):
                self.monitorAPIWS.postrobottomonitor(self.serial,
                                                    self.robot_on_poi,
                                                    self.map_name,
                                                    str(int(round(self.batt_percent,2)*100)),
                                                    str(self.robot_mode),
                                                    str(round(self.position_x,2)),
                                                    str(round(self.position_y,2)),
                                                    str(round(self.orientation_z,2)),
                                                    str(round(self.orientation_w,2)),
                                                    str(self.moving),
	                                                str(round(self.odo,0))
                                                    )
            else:
                rospy.loginfo("RP:%s",return_datapack)
                self.publish_state(return_datapack)

    def crcJK232(self,byteData):
        """
        Generate JK RS232 / RS485 CRC
        - 2 bytes, the verification field is "command code + length byte + data segment content",
        the verification method is thesum of the above fields and then the inverse plus 1, the high bit is in the front and the low bit is in the back.
        """
        CRC = 0
        for b in byteData:
            CRC += b
        crc_low = CRC & 0xFF
        crc_high = (CRC >> 8) & 0xFF
        return [crc_high, crc_low]   

def main(argv=sys.argv):

    rospy.init_node('Robot_monitor', anonymous=True)
    M = Monitor()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        M.monitor_data()
        rate.sleep()
    #rospy.spin()
    
    
    # rospy.shutdown()


if __name__ == '__main__':
    main()
