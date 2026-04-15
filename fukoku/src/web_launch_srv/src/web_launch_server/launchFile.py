#!/usr/bin/env python
import rospy
import rospkg
import roslaunch
import sys                                                                      
import roslib.packages

class LaunchFile(object):

    __pkg = None
    __file = None
    __launch = None
    __NeedProcess = False
    __isStarted = False
    __isCompleted = False
    __cmd = ''
    __args = []

    def __init__(self, name = 'rqt_gui', pkg='rqt_gui',file = 'rqt_gui'):
        self.__process_name = name
        self.__uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.__uuid)
        self.__pkg = pkg
        self.__file = file

        pass
    def isStated(self):
        return self.__isStarted
    def isCompleted(self):
        return self.__isCompleted
    def isNeedProcess(self):
        return self.__NeedProcess

    def setCommand(self, cmd, args=[]):

        if(self.__cmd != cmd):
            self.__NeedProcess = True
            self.__isCompleted = False
            self.__cmd = cmd
            self.__args = args
            rospy.loginfo ("setCommand %s : %s"%(self.__pkg,self.__cmd))
            rospy.loginfo (' '.join(map(str, self.__args)))

        
        pass    

    def doCommand(self):
        if(self.__NeedProcess):
            self.__NeedProcess = False
            if(not self.__isCompleted):
                if((self.__cmd == 'start' and  self.isStated() == False)):
                    self.start()
                elif (self.__cmd == 'start' and  self.isStated() == True):
                    self.__isCompleted = True
                elif((self.__cmd == 'stop' and  self.isStated() == True)):
                    self.stop()
                elif (self.__cmd == 'stop' and  self.isStated() == False):
                    self.__isCompleted = True
                elif (self.__cmd == 'run'):
                    self.run()
                    
        pass
        
    def run(self):
        rospy.loginfo ("run %s "%(self.__pkg))
        rospy.loginfo (' '.join(map(str, self.__args)))
        self.__launch = roslaunch.scriptapi.ROSLaunch()

 
        launch_files = [(self.__file, self.__args)]
        self.__launch = roslaunch.parent.ROSLaunchParent(self.__uuid, launch_files)
        self.__launch.start()


        rospy.loginfo ("run %s started"%(self.__pkg))
        self.__cmd = ''
        self.__isCompleted = True

        pass

    def start(self):
        """
        docstring
        """
        
        if(self.__isStarted == False):
            
            self.__isStarted = True
            self.__launch = roslaunch.scriptapi.ROSLaunch()
            # self.__launch = roslaunch.parent.ROSLaunchParent(self.__uuid, [self.__file])
            launch_files = [(self.__file, self.__args)]
            self.__launch = roslaunch.parent.ROSLaunchParent(self.__uuid, launch_files)
            self.__launch.start()
            rospy.loginfo ("launch %s started"%(self.__pkg))
            self.__isCompleted = True
            # rospy.sleep(2)
        pass


    def stop(self):
        """
        docstring
        """
        
        if(self.__isStarted == True):
            self.__isStarted = False
            self.__launch.shutdown()
            self.__launch = None
            rospy.loginfo ("shutdown %s"%(self.__pkg))
            self.__isCompleted = True
            # rospy.sleep(2)
        pass

        