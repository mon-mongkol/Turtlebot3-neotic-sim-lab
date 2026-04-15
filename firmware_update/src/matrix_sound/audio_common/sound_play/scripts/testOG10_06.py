#!/usr/bin/env python
import rospy
import os, sys 
import threading
import numpy as np
from enum import Enum
from std_msgs.msg import Int64, Int8
from sound_play.srv import SoundMode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sound_play.msg import SoundRequest
import time

from sound_play.libsoundplay import SoundClient

class sound_play:
    def __init__(self):
        self.counter = 0
       
        self.cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cbCmdVel)
        self.robot_mode = rospy.Subscriber("/aragorn/robot_mode", Int8, self.cbRobotMode)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.cbScan)
        self.sub_odom = rospy.Subscriber("/zlac706/odom", Odometry, self.cbOdom)
        self.sub_robot_status = rospy.Subscriber("/aragorn/base_status", 10)
        # self.sound_mode = rospy.Service("/sound_mode", SoundMode, self.cbSound)
        

        # self.current_vel = Twist()
        self.current_vel_li = 0.0
        self.current_vel_ang = 0.0
        self.current_robot_mode = 0

        self.moving = False
        self.cmd_hit = False
        self.period_cnd = Twist()
        self.period_state = Odometry()
        self.counter = 0

        self.SoundPlay = Enum('SoundPlay', 'ready_to_start uvc_on uvc_off completed go_to_docking_station')

        self.Robot_Mode = Enum('Robot_Mode', 'IDLE START_MOTOR SHUTDOWN_MOTOR SHUTDOWN_ROBOT EMERGENCY FUCN_1 DOWN_STAIRS TABLET_LOSS_COMMU UVC_ON UVC_OFF READY_TO_START DOCKING_MODE_ON DOCKING_MODE_OFF')

        loop_rate = rospy.Rate(50)

        self.soundhandle = SoundClient()
        self.scan_data = LaserScan()
        self.data_zone = LaserScan()
        self.data_zone.ranges = [ 0.0, 0.0, 0.0, 0.0, 0.998000, 0.998000, 0.998000, 0.998000, 0.992000, 0.992000, 0.992000, 0.992000, 0.988000, 0.982000, 0.978000, 0.976000, 0.976000, 0.972000, 0.968000, 0.964000, 0.960000, 0.960000, 0.958000, 0.954000, 0.950000, 0.948000, 0.948000, 0.946000, 0.942000, 0.942000, 0.940000, 0.936000, 0.936000, 0.930000, 0.926000, 0.922000, 0.920000, 0.920000, 0.918000, 0.912000, 0.910000, 0.908000, 0.906000, 0.906000, 0.902000, 0.900000, 0.896000, 0.896000, 0.896000, 0.892000, 0.890000, 0.886000, 0.884000, 0.884000, 0.882000, 0.880000, 0.878000, 0.876000, 0.874000, 0.874000, 0.872000, 0.870000, 0.866000, 0.864000, 0.864000, 0.862000, 0.860000, 0.860000, 0.858000, 0.858000, 0.854000, 0.854000, 0.852000, 0.850000, 0.850000, 0.848000, 0.846000, 0.846000, 0.842000, 0.842000, 0.842000, 0.842000, 0.842000, 0.842000, 0.842000, 0.836000, 0.834000, 0.832000, 0.832000, 0.832000, 0.832000, 0.828000, 0.828000, 0.828000, 0.828000, 0.826000, 0.824000, 0.824000, 0.824000, 0.824000, 0.822000, 0.822000, 0.818000, 0.816000, 0.818000, 0.818000, 0.816000, 0.814000, 0.812000, 0.812000, 0.812000, 0.812000, 0.812000, 0.812000, 0.810000, 0.810000, 0.810000, 0.810000, 0.808000, 0.808000, 0.808000, 0.808000, 0.808000, 0.806000, 0.806000, 0.806000, 0.806000, 0.806000, 0.806000, 0.806000, 0.806000, 0.804000, 0.802000, 0.810000, 0.808000, 0.808000, 0.800000, 0.802000, 0.800000, 0.800000, 0.800000, 0.798000, 0.798000, 0.798000, 0.798000, 0.798000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.796000, 0.796000, 0.796000, 0.798000, 0.798000, 0.796000, 0.796000, 0.796000, 0.798000, 0.798000, 0.798000, 0.798000, 0.798000, 0.798000, 0.798000, 0.798000, 0.798000, 0.800000, 0.800000, 0.800000, 0.802000, 0.802000, 0.800000, 0.802000, 0.802000, 0.802000, 0.804000, 0.804000, 0.804000, 0.804000, 0.804000, 0.804000, 0.806000, 0.806000, 0.806000, 0.806000, 0.806000, 0.808000, 0.810000, 0.810000, 0.810000, 0.810000, 0.812000, 0.810000, 0.810000, 0.810000, 0.812000, 0.814000, 0.816000, 0.816000, 0.816000, 0.818000, 0.820000, 0.820000, 0.820000, 0.820000, 0.822000, 0.824000, 0.824000, 0.824000, 0.826000, 0.828000, 0.828000, 0.828000, 0.828000, 0.830000, 0.832000, 0.834000, 0.834000, 0.836000, 0.836000, 0.838000, 0.840000, 0.840000, 0.842000, 0.844000, 0.846000, 0.846000, 0.846000, 0.848000, 0.850000, 0.862000, 0.868000, 0.868000, 0.862000, 0.860000, 0.860000, 0.860000, 0.860000, 0.862000, 0.864000, 0.864000, 0.866000, 0.866000, 0.868000, 0.872000, 0.874000, 0.876000, 0.876000, 0.880000, 0.882000, 0.882000, 0.882000, 0.884000, 0.888000, 0.890000, 0.892000, 0.892000, 0.896000, 0.898000, 0.902000, 0.902000, 0.902000, 0.904000, 0.908000, 0.910000, 0.912000, 0.912000, 0.916000, 0.920000, 0.922000, 0.922000, 0.926000, 0.930000, 0.932000, 0.934000, 0.934000, 0.938000, 0.942000, 0.944000, 0.946000, 0.946000, 0.952000, 0.956000, 0.958000, 0.958000, 0.960000, 0.964000, 0.970000, 0.974000, 0.974000, 0.976000, 0.982000, 0.986000, 0.986000, 0.990000, 0.994000, 0.996000, 1.000000, 1.000000, 1.006000, 1.010000, 1.014000, 1.020000, 1.020000, 1.024000, 1.030000, 1.034000, 1.034000, 1.038000, 1.044000, 1.050000, 1.054000, 1.054000, 1.060000, 1.070000, 1.068000, 1.062000, 1.062000, 1.064000, 1.058000, 1.052000, 1.044000, 1.044000, 1.040000, 1.032000, 1.028000, 1.024000, 1.018000, 1.018000, 1.012000, 1.006000, 1.002000, 0.998000, 0.998000, 0.994000, 0.988000, 0.982000, 0.978000, 0.974000, 0.974000, 0.968000, 0.964000, 0.960000, 0.960000, 0.954000, 0.950000, 0.948000, 0.942000, 0.940000, 0.940000, 0.936000, 0.930000, 0.930000, 0.926000, 0.926000, 0.922000, 0.918000, 0.912000, 0.908000, 0.904000, 0.904000, 0.904000, 0.900000, 0.898000, 0.894000, 0.894000, 0.888000, 0.886000, 0.882000, 0.880000, 0.880000, 0.878000, 0.874000, 0.870000, 0.866000, 0.866000, 0.866000, 0.862000, 0.858000, 0.856000, 0.854000, 0.854000, 0.850000, 0.848000, 0.846000, 0.846000, 0.844000, 0.844000, 0.844000, 0.846000, 0.842000, 0.836000, 0.836000, 0.834000, 0.832000, 0.830000, 0.826000, 0.826000, 0.824000, 0.822000, 0.820000, 0.818000, 0.816000, 0.816000, 0.812000, 0.810000, 0.808000, 0.808000, 0.808000, 0.804000, 0.802000, 0.800000, 0.800000, 0.800000, 0.798000, 0.796000, 0.792000, 0.790000, 0.790000, 0.788000, 0.788000, 0.786000, 0.784000, 0.784000, 0.782000, 0.782000, 0.780000, 0.778000, 0.778000, 0.776000, 0.774000, 0.774000, 0.774000, 0.774000, 0.772000, 0.770000, 0.768000, 0.768000, 0.766000, 0.766000, 0.764000, 0.762000, 0.762000, 0.760000, 0.760000, 0.760000, 0.758000, 0.758000, 0.756000, 0.756000, 0.756000, 0.754000, 0.752000, 0.752000, 0.752000, 0.750000, 0.750000, 0.750000, 0.750000, 0.750000, 0.748000, 0.748000, 0.746000, 0.746000, 0.746000, 0.744000, 0.742000, 0.742000, 0.742000, 0.742000, 0.742000, 0.742000, 0.740000, 0.740000, 0.738000, 0.738000, 0.738000, 0.738000, 0.738000, 0.736000, 0.736000, 0.738000, 0.736000, 0.736000, 0.736000, 0.736000, 0.734000, 0.734000, 0.734000, 0.734000, 0.734000, 0.734000, 0.734000, 0.734000, 0.734000, 0.734000, 0.732000, 0.732000, 0.732000, 0.734000, 0.734000, 0.734000, 0.734000, 0.736000, 0.744000, 0.744000, 0.742000, 0.734000, 0.734000, 0.734000, 0.734000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.732000, 0.734000, 0.734000, 0.734000, 0.734000, 0.734000, 0.736000, 0.736000, 0.736000, 0.736000, 0.736000, 0.736000, 0.738000, 0.738000, 0.738000, 0.738000, 0.738000, 0.740000, 0.740000, 0.740000, 0.742000, 0.742000, 0.742000, 0.742000, 0.744000, 0.744000, 0.744000, 0.744000, 0.746000, 0.746000, 0.746000, 0.746000, 0.748000, 0.748000, 0.750000, 0.750000, 0.750000, 0.752000, 0.752000, 0.752000, 0.752000, 0.754000, 0.756000, 0.756000, 0.758000, 0.758000, 0.758000, 0.760000, 0.762000, 0.764000, 0.764000, 0.766000, 0.766000, 0.766000, 0.766000, 0.768000, 0.768000, 0.770000, 0.770000, 0.772000, 0.774000, 0.776000, 0.778000, 0.778000, 0.778000, 0.778000, 0.780000, 0.782000, 0.782000, 0.784000, 0.786000, 0.786000, 0.790000, 0.790000, 0.792000, 0.792000, 0.794000, 0.794000, 0.798000, 0.798000, 0.800000, 0.802000, 0.802000, 0.804000, 0.806000, 0.808000, 0.812000, 0.812000, 0.814000, 0.814000, 0.818000, 0.820000, 0.820000, 0.822000, 0.830000, 0.838000, 0.838000, 0.838000, 0.836000, 0.838000, 0.838000, 0.838000, 0.842000, 0.844000, 0.848000, 0.848000, 0.850000, 0.850000, 0.854000, 0.854000, 0.858000, 0.860000, 0.862000, 0.866000, 0.866000, 0.868000, 0.870000, 0.874000, 0.876000, 0.876000, 0.880000, 0.882000, 0.890000, 0.890000, 0.894000, 0.896000, 0.900000, 0.904000, 0.904000, 0.910000, 0.914000, 0.912000, 0.912000, 0.918000, 0.920000, 0.926000, 0.930000, 0.930000, 0.932000, 0.936000, 0.940000, 0.944000, 0.944000, 0.948000, 0.952000, 0.956000, 0.962000, 0.962000, 0.968000, 0.972000, 0.976000, 0.980000, 0.980000, 0.986000, 0.990000, 0.994000, 0.994000, 1.000000, 1.004000, 1.010000, 1.016000, 1.016000, 1.022000, 1.026000, 1.030000, 1.030000, 1.038000, 1.044000, 1.050000, 1.056000, 1.056000, 1.062000, 1.068000, 1.066000, 1.064000, 1.064000, 1.062000, 1.054000, 1.048000, 1.044000, 1.044000, 1.040000, 1.034000, 1.028000, 1.024000, 1.020000, 1.020000, 1.016000, 1.012000, 1.008000, 1.004000, 1.004000, 0.998000, 0.994000, 0.990000, 0.986000, 0.986000, 0.982000, 0.976000, 0.974000, 0.970000, 0.970000, 0.966000, 0.962000, 0.958000, 0.956000, 0.956000, 0.952000, 0.948000, 0.946000, 0.942000, 0.938000, 0.938000, 0.934000, 0.932000, 0.930000, 0.930000, 0.930000, 0.926000, 0.922000, 0.918000, 0.914000, 0.914000, 0.912000, 0.910000, 0.908000, 0.904000, 0.904000, 0.904000, 0.898000, 0.894000, 0.894000, 0.890000, 0.890000, 0.890000, 0.886000, 0.882000, 0.882000, 0.882000, 0.880000, 0.876000, 0.874000, 0.872000, 0.870000, 0.870000, 0.868000, 0.866000, 0.864000, 0.862000, 0.862000, 0.860000, 0.858000, 0.856000, 0.856000, 0.856000, 0.854000, 0.858000, 0.866000, 0.860000, 0.860000, 0.846000, 0.844000, 0.842000, 0.840000, 0.840000, 0.840000, 0.838000, 0.836000, 0.834000, 0.832000, 0.832000, 0.830000, 0.828000, 0.828000, 0.828000, 0.828000, 0.826000, 0.824000, 0.824000, 0.822000, 0.822000, 0.822000, 0.820000, 0.818000, 0.818000, 0.816000, 0.816000, 0.816000, 0.814000, 0.812000, 0.812000, 0.812000, 0.810000, 0.810000, 0.810000, 0.808000, 0.808000, 0.808000, 0.806000, 0.806000, 0.806000, 0.806000, 0.804000, 0.802000, 0.802000, 0.802000, 0.802000, 0.802000, 0.800000, 0.798000, 0.800000, 0.800000, 0.800000, 0.798000, 0.798000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.798000, 0.798000, 0.796000, 0.794000, 0.794000, 0.796000, 0.796000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.792000, 0.792000, 0.794000, 0.792000, 0.792000, 0.792000, 0.792000, 0.794000, 0.794000, 0.794000, 0.792000, 0.792000, 0.794000, 0.794000, 0.794000, 0.794000, 0.794000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.796000, 0.798000, 0.800000, 0.800000, 0.800000, 0.800000, 0.800000, 0.802000, 0.802000, 0.814000, 0.820000, 0.810000, 0.806000, 0.806000, 0.806000, 0.808000, 0.808000, 0.808000, 0.808000, 0.808000, 0.808000, 0.810000, 0.810000, 0.810000, 0.812000, 0.812000, 0.810000, 0.814000, 0.814000, 0.816000, 0.814000, 0.816000, 0.816000, 0.816000, 0.820000, 0.820000, 0.822000, 0.822000, 0.824000, 0.826000, 0.826000, 0.826000, 0.826000, 0.828000, 0.828000, 0.830000, 0.830000, 0.830000, 0.836000, 0.838000, 0.836000, 0.838000, 0.838000, 0.838000, 0.840000, 0.842000, 0.844000, 0.844000, 0.846000, 0.848000, 0.848000, 0.848000, 0.850000, 0.852000, 0.854000, 0.856000, 0.856000, 0.858000, 0.860000, 0.860000, 0.864000, 0.864000, 0.866000, 0.868000, 0.870000, 0.874000, 0.874000, 0.876000, 0.876000, 0.878000, 0.882000, 0.882000, 0.886000, 0.888000, 0.890000, 0.890000, 0.892000, 0.894000, 0.898000, 0.902000, 0.902000, 0.904000, 0.908000, 0.910000, 0.912000, 0.912000, 0.914000, 0.918000, 0.922000, 0.922000, 0.924000, 0.928000, 0.930000, 0.934000, 0.934000, 0.936000, 0.940000, 0.944000, 0.948000, 0.948000, 0.956000, 0.960000, 0.958000, 0.958000, 0.958000, 0.962000, 0.964000]

        self._isMoving = False
        self._isBackward = False
        self._isNearObstacle = False
        self._isInteruptSRV = False
        self.time_stamp_cmd = 0

        # threads = []
        # t1 = threading.Thread(target=self.compairScan)
        # threads.append(t1)
        # t1.start()

        # for t in threads:
        #     t.start()

        while not rospy.is_shutdown():
            # print("aaa")
            self.soundUpdate()
            loop_rate.sleep()


    
    def compairScan(self):
        # print("comapire")
        if len(self.scan_data.ranges) == 1024:
            for i in range(1, 1000):
                if(self.scan_data.ranges[i] <= self.data_zone.ranges[i]):
                    self._isNearObstacle = True
                    break
                else:
                    self._isNearObstacle = False
            
            # print(self._isNearObstacle)


    def cbScan(self, msg):
        # prwint("comapire")
        self.scan_data = msg
        for i in range(6, len(self.scan_data.ranges)):
                if(self.scan_data.ranges[i] <= (self.data_zone.ranges[i])):
                    self._isNearObstacle = True
                    break
                else:
                    self._isNearObstacle = False
                
        
        # print(self._isNearObstacle)
            
            

    def cbCmdVel(self, msg):
        # self.current_vel = msg
        # if (self.period_cnd.linear.x != msg.linear.x) or (self.period_cnd.angular.z != msg.angular.z):
        if (self.period_cnd.linear.x != 0.0) or (self.period_cnd.angular.z != 0.0):
            self.cmd_hit = True
        else:
            self.cmd_hit = False

        self.period_cnd = msg
        self.time_stamp_cmd = rospy.Time.from_sec(time.time()).to_nsec()
        
        # print(self.cmd_hit)
        # print(self.current_vel)

    def cbOdom(self, msg):
        # self.current_vel_li = abs(msg.twist.twist.linear.x)
        # self.current_vel_ang = abs(msg.twist.twist.angular.z)
        if (self.period_state.pose.pose.position.x != msg.pose.pose.position.x) or (self.period_state.pose.pose.position.y != msg.pose.pose.position.y):
            if (self.period_state.twist.twist.linear.x != msg.twist.twist.linear.x) or (self.period_state.twist.twist.angular.z != msg.twist.twist.angular.z):
                self.counter += 1
                if self.counter >= 3:
                    self.moving = True 
        else:
            self.counter = 0
            self.moving = False
	    # print(self.moving)
        self.period_state = msg
    
    def cbRobotMode(self, msg):
        # self.current_robot_mode = msg.data
        if self.current_robot_mode != msg.data:

            self._isInteruptSRV = True

            if msg.data == self.Robot_Mode.READY_TO_START.value:
                # self.soundhandle.say('Aragorn ready to start!')
                self.soundhandle.playWave('Aragorn ready to start!.mp3')
            elif msg.data == self.Robot_Mode.UVC_ON.value:
                pass
                # self.soundhandle.say('Turn on U-V-C lamp')
                # self.soundhandle.playWave('Turn on UV-C lamps.mp3')
            elif msg.data == self.Robot_Mode.UVC_OFF.value:
                pass
                # self.soundhandle.say('Turn off U-V-C lamp')
                # self.soundhandle.playWave('Turn off UV-C lamps.mp3')
            elif msg.data == self.Robot_Mode.START_MOTOR.value:
                pass
                # self.soundhandle.say('Ready to drive to destination!')
                self.soundhandle.playWave('Ready to drive to destination!.mp3')
            elif msg.data == self.Robot_Mode.EMERGENCY.value:
                # self.soundhandle.say('Warning emergency active!')
                self.soundhandle.playWave('Warning emergency active!.mp3')
            elif msg.data == self.Robot_Mode.TABLET_LOSS_COMMU.value:
                pass
                # self.soundhandle.say('Tablet disconnected')
                # self.soundhandle.playWave('Tablet disconnected.mp3')
            elif msg.data == self.Robot_Mode.DOCKING_MODE_ON.value:
                pass
                # self.soundhandle.say('Backward to docking station')
                # self.soundhandle.playWave('Backward to docking station.mp3')
            else:
                pass

            # sound when robot moving
            if (msg.data == self.Robot_Mode.START_MOTOR.value) or (msg.data == self.Robot_Mode.UVC_OFF.value) or (msg.data ==self.Robot_Mode.DOCKING_MODE_OFF.value):
                self._isMoving = True
                self._isBackward = False
            elif msg.data == self.Robot_Mode.DOCKING_MODE_ON.value:
                self._isMoving = False
                self._isBackward = True
            elif msg.data == self.Robot_Mode.EMERGENCY.value:
                self._isMoving = False
                self._isBackward = False
            else:
                self._isMoving = False
                self._isBackward = False
            
            self._isInteruptSRV = False
            self.current_robot_mode = msg.data
        else:
            pass

        

        # print(msg.data, self._isMoving, self._isBackward)

        
    # def cbSound(self, req):

    #     return False

    def soundUpdate(self):

        if self._isInteruptSRV == False:

            if (self._isMoving == True) and (self._isNearObstacle == True):
                if (self.moving == True) and (self.cmd_hit == True):
                    # print("on sound near obstacle")
                    print("on sound near obj")
                    self.soundhandle.playWave('move_new.wav')
                    self.sleep(0.3)

                else:
                    pass
                    # print("mute sound")
                
            elif (self._isMoving == True) and (self._isNearObstacle == False):
                if (self.moving == True) and (self.cmd_hit == True):
                    print("on sound moving")
                    self.soundhandle.playWave('moveing.wav')
                    self.sleep(1.6)
                else:
                    pass
                    print("mute sound")

            elif (self._isBackward == True):
                self.soundhandle.playWave('Reverse.wav')
                self.sleep(0.6)
                # print("on sound back ward")
            
            else:
                pass
                print("else else ++++++")
                # if (self.moving == True) and (self.cmd_hit == True):
                #     # print("on sound moving")
                #     self.soundhandle.playWave('moveing.wav')
                #     self.sleep(1.5)
                # else:
                #     pass
                #     # print("mute sound")
        else:
            pass
        
        if((rospy.Time.from_sec(time.time()).to_nsec() - self.time_stamp_cmd) >= 5000000000):
            self.moving = False
            self.cmd_hit = False
        else:
            pass

        print(self.moving, self.cmd_hit)

    def sleep(self, t):
        try:
            rospy.sleep(t)
        except:
            pass

if __name__ == '__main__':
    rospy.init_node('number_counter')
    sound_play()
    rospy.spin()

