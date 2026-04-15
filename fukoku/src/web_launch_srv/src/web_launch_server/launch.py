#!/usr/bin/env python
import rospy
import rospkg
import roslaunch

class Launch(object):
    __instance = None
    __process_name = ""
    __cmd = ''
    __isStarted = False
    __isCompleted = False
    

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(Launch, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def __init__(self):
        pass

    def __del__(self):
        pass

    @property
    def name(self):
        return self.__process_name

    def setCommand(self, cmd):
        if(self.__cmd != cmd):
            self.__isCompleted = False
            self.__cmd = cmd
        
        pass

    def doCommand(self):
        if((self.__cmd == 'start' and  self.isStated() == False) or self.isStated()):
            self.start()
            self.__isCompleted = True
        elif((self.__cmd == 'stop' and  self.isStated() == True) or  not self.isStated()):
            self.stop()
            self.__isCompleted = True

            
        pass

    def start(self):
        self.__isStarted = True
        pass

    def stop(self):
        self.__isStarted = False
        pass
    def isStated(self):
        return self.__isStarted
    def isCompleted(self):
        return self.__isCompleted
