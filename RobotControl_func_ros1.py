# import pythoncom
# pythoncom.CoInitialize()

import time
import random
from tkinter.tix import Tree
import numpy as np
from PyQt5.QtCore import QMutex, QObject, pyqtSlot, QThreadPool, QRunnable, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication
import math

import rospy
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../../', 'tm_msgs/msg'))
# print(sys.path)
# from tm_msgs import msg
from tm_msgs.msg import *
from tm_msgs.srv import *

# Chong: for matrix Rotation  因為這隻手必是斜45度
cos45 = math.cos(math.radians(45))
sin45 = math.sin(math.radians(45))
R = np.array([[cos45,-sin45,0],[sin45,cos45,0],[0,0,1]])

mutex = QMutex()
# Robot Arm move
class myMitter(QObject):
    done = pyqtSignal(bool)

class worker(QThread):
        def __init__(self, pos, speed, line):
            super(worker, self).__init__()
            self.pos = pos
            self.speed = speed
            self.line = line
            self.mitter = myMitter()

        @pyqtSlot()
        def run(self):
            try:
                mutex.lock()
                rospy.wait_for_service('tm_driver/ask_sta')
                rospy.wait_for_service('tm_driver/set_event')
                rospy.wait_for_service('tm_driver/set_positions')
                ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)
                set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
                set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)
                print(self.pos)


                if self.line == False:
                    set_positions(SetPositionsRequest.PTP_T, self.pos, self.speed, 1, 0, False)
                else:
                    set_positions(SetPositionsRequest.LINE_T, self.pos, self.speed, 0.5, 0, False)

                set_event(SetEventRequest.TAG, 1, 0)

                while True:
                    rospy.sleep(0.2)
                    res = ask_sta('01', str(1), 1)
                    if res.subcmd == '01':
                        data = res.subdata.split(',')
                        if data[1] == 'true':
                            rospy.loginfo('point %d (Tag %s) is reached', 1, data[0])
                            break
              
            except Exception as e: 
                print(e)


            # self.emitter.done.emit(False)
            self.mitter.done.emit(True)
            # print('emit')

            mutex.unlock()

class RobotControl_Func():
    def __init__(self):
        super().__init__()

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.Rx = 0
        self.Ry = 0
        self.Rz = 0
        self.speed = 0
        self.accel = 0

        self.pool = QThreadPool.globalInstance()
        # For Arm
        self.pool.setMaxThreadCount(1)
        # For Arm + AMM
        # self.pool.setMaxThreadCount(2) 
        self.threadDone = True


    def exit(self):
        set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
        set_event(SetEventRequest.EXIT, 1, 0)
    def stop(self):
        set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
        set_event(SetEventRequest.STOP, 1, 0)
    def resume(self):
        set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
        set_event(SetEventRequest.RESUME,1,0)
    def pasue(self):
        set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
        set_event(SetEventRequest.PAUSE, 1, 0)
    def on_worker_done(self, threadDone):
        #print(threadDone)
        print("True")
        self.threadDone = True

    def set_TMPos(self, pos, speed = 1, line = False):
        # transself.set_TMPos_new(pos)form to TM robot coordinate
        tmp = []
        runnable = worker(tmp, speed, line)

        ori_xyz = [pos[0],pos[1],pos[2]]
        after_xyz = ori_xyz @ R
        tmp.append(after_xyz[0] / 1000)
        tmp.append(after_xyz[1] / 1000)
        tmp.append(after_xyz[2] / 1000)
        #tmp.append(after_xyz[0]/1000)
        #tmp.append(after_xyz[1]/1000)
        #tmp.append(after_xyz[2]/1000)
        tmp.append(pos[3] * np.pi / 180)
        tmp.append(pos[4] * np.pi / 180)
        tmp.append(pos[5] * np.pi / 180)

        self.threadDone = False
        runnable = worker(tmp, speed, line)
        runnable.mitter.done.connect(self.on_worker_done)
        runnable.start()

        runnable.wait()        
        

    def get_TMPos(self):
        # listen to 'feedback_states' topic
        data = rospy.wait_for_message("/feedback_states", FeedbackState, timeout=None)
        print(data.tool_pose)
        print(data.tcp_speed)
        cos45 = math.cos(math.radians(-45))
        sin45 = math.sin(math.radians(-45))
        R2 = np.array([[cos45,-sin45,0],[sin45,cos45,0],[0,0,1]])
        
        current_pos = list(data.tool_pose)
        ori_xyz = [current_pos[0],current_pos[1],current_pos[2]]
        after_xyz = ori_xyz @ R2
        
        current_pos[0] = after_xyz[0] * 1000
        current_pos[1] = after_xyz[1] * 1000
        current_pos[2] = after_xyz[2] * 1000

        #current_pos[0] = current_pos[0] * 1000
        #current_pos[1] = current_pos[1] * 1000
        #current_pos[2] = current_pos[2] * 1000
        current_pos[3] = current_pos[3] * 180 / np.pi  #出來是角度
        current_pos[4] = current_pos[4] * 180 / np.pi
        current_pos[5] = current_pos[5] * 180 / np.pi
        # print(self.robot)
        return current_pos
