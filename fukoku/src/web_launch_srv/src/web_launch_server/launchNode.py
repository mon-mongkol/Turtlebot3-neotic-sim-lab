#!/usr/bin/env python
import rospy
import rospkg
import roslaunch
import sys                                                                      


from .launch import Launch 


class LaunchNode(Launch):
    __instance = None

    __package = None
    __executable = None
    __node = None
    __launch = None
    __process = None

    # def __new__(cls, *args, **kwargs):
    #     if not cls.__instance:
    #         cls.__instance = super(LaunchNode, cls).__new__(cls, *args, **kwargs)
    #     return cls.__instance

    def __init__(self,package='rqt_gui',executable = 'rqt_gui'):
        self.__package = package
        self.__process_name = self.__package
        self.__executable = executable
        self.__node = roslaunch.core.Node(self.__package, self.__executable)
        self.__launch = roslaunch.scriptapi.ROSLaunch()
        self.__launch.start()
        pass

    def start(self):
        """
        docstring
        """
        super(LaunchNode,self).start()
        self.__process = self.__launch.launch(self.__node)
        print ("%s is alive : %d"%(self.__package ,self.__process.is_alive()))
        pass

    def isAlive(self, parameter_list):
        """
        docstring
        """
        return self.__process.is_alive()

    def stop(self):
        """
        docstring
        """
        super(LaunchNode,self).stop()
        self.__process.stop()
        pass

        