#! /usr/bin/env python3

import sys
import enum
import requests
import socket
import json
import urllib3
import time
import socketio
import asyncio

class monitorAPIWS(socketio.AsyncClient):

    sio = socketio.Client(reconnection=True, reconnection_attempts=0, reconnection_delay=1, reconnection_delay_max=5, randomization_factor=0.5, logger=False)
    ip_port = ""
    connected = False
    def setup(self):
        self.call_backs()
        # print(self.ip_port)
        try:
            self.sio.connect(self.ip_port or 'http://localhost:5001',namespaces=['/monitor'],transports=["websocket"])
            self.connected = True
        except Exception as e:
            print(e) 

        self.postrobottomonitor_data = {}
        self.postrobottomonitorerror_data = {}
        # self.getall_data = {}
        # self.getmap_data ={}
        # self.listmap_data = {}

    def call_backs(self):
        @self.sio.event
        def connect():
            self.sio.emit('connect','connect')
            print('connection established')

        @self.sio.event
        def auth(data):
            print(f"Data Received {data}")

        @self.sio.event
        def disconnect():
            print('disconnected from server')

        @self.sio.on('postrobottomonitor',namespace='/monitor')
        def response(data):
            # print(data)  # {'from': 'server'}
            self.postrobottomonitor_data = data

        @self.sio.on('postrobottomonitorerror',namespace='/monitor')
        def response(data):
            # print(data)  # {'from': 'server'}
            self.postrobottomonitorerror_data = data

        @self.sio.on('getall',namespace='/monitor')
        def response(data):
            # print(data)  # {'from': 'server'}
            self.getall_data = data

        @self.sio.on('listmap',namespace='/monitor')
        def response(data):
            # print(data)  # {'from': 'server'}
            self.listmap_data = data

        @self.sio.on('getmap',namespace='/monitor')
        def response(data):
            # print(data)  # {'from': 'server'}
            self.getmap_data = data

        # @self.sio.on('stop')
        # def response(data):
        #     # print(data)  # {'from': 'server'}
        #     print(data)

    def set_ip(self,ip):
            self.ip_port = ip
            # self.loop() 

    def run(self):
            self.setup()
            # self.loop() 

    def postrobottomonitor(self, robotname , poi , map_name , batt_percent, robot_mode , pos_x , pos_y , ori_z , ori_w , moving, odo):
        # print("EMIT postrobottomonitor" )
        # print(robotname, poi , map_name , batt_percent, robot_mode , pos_x , pos_y , ori_z , ori_w , moving ,odo)
        try:
            self.sio.emit("postrobottomonitor", {
                "robotname":robotname,
                "poi":poi,
                "map_name":map_name,
                "batt_percent":batt_percent,
                "robot_mode":robot_mode,
                "pos_x":pos_x,
                "pos_y":pos_y,
                "ori_z":ori_z,
                "ori_w":ori_w,
                "moving":moving,
                "odo":odo
                },namespace='/monitor')
        except Exception as e:
            print(e)
    def postrobottomonitorerror(self, robotname ,error):
        print("EMIT postrobottomonitorerror" )
        try:
            self.sio.emit("postrobottomonitorerror", {
                "robotname":robotname,
                "error":error
                },namespace='/monitor')  
        except Exception as e:
            print(e)    

    # def getall(self):
    #     self.sio.emit('getall',namespace='/monitor')    

    # def listmap(self):
    #     self.sio.emit('listmap',namespace='/monitor')   

    # def getmap(self,map_name):
    #     self.sio.emit('getmap',{"map_name":map_name},namespace='/monitor')   

    # def stop(self):
    #     self.sio.emit('stop')   
