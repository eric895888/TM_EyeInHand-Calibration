import os
import sys
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from ui.mainwindow import Ui_MainWindow
import pyrealsense2 as rs
import numpy as np
import cv2
# from robot.controller import Controller
# from robot.gripper import Gripper
import time
import EIHCali
from gripper import Gripper 
import rospy
import RobotControl_func_ros1 as RobotControl_func
import shutil
#to do
#顯示的影像還沒校正失真


#__file__是 Python 中一个内置變數,表示絕對路徑
#'../' 上一層目錄
sys.path.append(os.path.dirname(__file__) + os.sep)  #os.sep 表示当前操作系统的路径分隔符，如在 Windows 上是 \，在 Unix 上是 /。
print(sys.path[0])

# def StrToNdArray(IncomeStr):
#     if IncomeStr.strip()[:1] == '[' and IncomeStr.strip()[-1:] == ']': 
#         if IncomeStr.strip()[1:].strip()[:1] == '[' and IncomeStr.strip()[:-1].strip()[-1:] == ']': #[[]]
#             # print('二维数组')
#             type = 2
#         else: #[]
#             # print('一维数组')
#             type = 1
#     else:
#         type = 0
#         print('不可处理的字符串')
#     # print('type: '+str(type)+'\n')
#     if(type == 1):
#         OutArray = IncomeStr.strip()[1:-1].split()
#         return np.array(OutArray)
#     if(type == 2):
#         Intermediate = IncomeStr.strip()[1:-1].split('\n')
#         h = len(StrToNdArray(Intermediate[0]))
#         # print("H:"+str(h)+'\n')
#         OutArray = np.empty(shape=[0, h], dtype=float)
#         for x in Intermediate:
#             # print(str(StrToArray(x)) + '\n--------------')
#             OutArray = np.r_[OutArray, StrToNdArray(x).reshape((1,h)) ]
#         return OutArray
#     if(type !=1 and type != 2):
#         return False
    
#Error occured during execution of the processing block! See the log for more info 代表realsense的thread被block
class RGBDThread(QThread):
    rgbd_trigger = pyqtSignal(object)
    def __init__(self):
        super(RGBDThread, self).__init__()
        self._mutex = QMutex()
        self._running = True
        self.pipeline = rs.pipeline()
        #self.colorizer = rs.colorizer()
        self.config = rs.config()
        #self.config.enable_device('f0265339') #只有一隻相機就不用指定
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        profile = self.pipeline.start(self.config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()  
        print("Depth units setted: ", '%.5f'%self.depth_scale)  #realsense 400系列預設是0.001
        self.scale = 1
    def __del__(self):
        self.pipeline.stop()
        self.wait()
    def run(self): #去頭去尾 [self.color_image,depth_image_3c,self.depth_image] 代表rgb,三通道的depth_map,原本的depth_map
            try:
                while self.running():
                    # Wait for a coherent pair of frames: depth and color
                    try:
                        frames = self.pipeline.wait_for_frames()
                        aligned_frames = self.align.process(frames)
                        depth_frame = aligned_frames.get_depth_frame()
                        color_frame = aligned_frames.get_color_frame()
                        # color_frame = frames.get_color_frame()
                        # depth_frame = frames.get_depth_frame()
                        if not depth_frame or not color_frame:
                            print("continue")
                            continue
                    except Exception as e:
                        print(str(e))
                        pass
                    self.d_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    # Convert images to numpy arrays
                    self.depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.uint8(color_frame.get_data())
                    
                    self.color_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2RGB)
                    #depth_image_3c = cv2.cvtColor(depth_image_3c,cv2.COLOR_BGR2RGB)
                    depth_map_normalized = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U) #normalize 0~255
                    #height, width = depth_map_normalized.shape
                    
                    self.rgbd_pair = [self.color_image,depth_map_normalized]
                    self.rgbd_trigger.emit(self.rgbd_pair)
                    # cv2.waitKey(5)
            except NameError as e:
                print(e)
                self.pipeline.stop()
    def pause(self):
        print("pause streaming")
        self._mutex.lock()
        self._running = False
        # self.pipeline.stop()
        self._mutex.unlock()
    def restart(self):
        global mode
        mode = 2  #TODO: 停止绘制 BBox和 Point
        print("restart streaming")
        self._mutex.lock()
        self._running = True
        # self.pipeline.start()
        self._mutex.unlock()
        self.start()
    def running(self):
        try:
            self._mutex.lock()
            return self._running
        finally:
            self._mutex.unlock()
    def getDepth(self): #?功能
        try:
            # if realX > releaseX:
            #     x1 = releaseX
            #     x2 = realX
            # else:
            #     x2 = releaseX
            #     x1 = realX
            # if realY > releaseY:
            #     y1 = releaseY
            #     y2 = realY
            # else:
            #     y2 = releaseY
            #     y1 = realY
            # scaleX1 = int((1.3 * x1 + 0.7 * x2) / 2 + 0.5)
            # scaleX2 = int((0.7 * x1 + 1.3 * x2) / 2 + 0.5)
            # scaleY1 = int((1.3 * y1 + 0.7 * y2) / 2 + 0.5)
            # scaleY2 = int((0.7 * y1 + 1.3 * y2) / 2 + 0.5)
            # # print(scaleX1, scaleX2, scaleY1, scaleY2)
            frames = self.pipeline.wait_for_frames()
            # New_depth_image = np.asanyarray(self.align.process(frames).get_depth_frame().get_data())
            # # # print(np.shape(New_depth_image))
            # # # plt.imshow(New_depth_image)
            # # # plt.show()
            # # depthmap = New_depth_image[scaleY1:scaleY2, scaleX1:scaleX2].astype(float) * self.depth_scale * 100
            # # depthmap = depthmap.flatten()
            # # # print("-----------BBox有效点-------------\n", np.shape(depthmap))
            # # # TODO: 去除异常值 0
            # # indices = np.where(depthmap == 0.0)
            # # depthmap = np.delete(depthmap, indices)
            # # # print("-----------去除0后大小-------------\n",np.shape(depthmap))
            # # # TODO: 去除异常值 保留25%~75%之间的数字
            # # Outliers1 = np.percentile(depthmap, 25)
            # # Outliers2 = np.percentile(depthmap, 75)
            # # # print(Outliers1,Outliers2)
            # # indices = np.where(depthmap > Outliers2)
            # # depthmap = np.delete(depthmap, indices)
            # # indices = np.where(depthmap < Outliers1)
            # # depthmap = np.delete(depthmap, indices)
            # # # print("-----------删除异常值后-------------\n",np.shape(depthmap))
            # # dist,_,_,_ = cv2.mean(depthmap)
            # depth = New_depth_image[realY, realX].astype(float) * self.depth_scale * 1000   #因為存成ndarray所以變成y,x
            depth=self.align.process(frames).get_depth_frame().get_distance(realX, realY)*1000
            return "Depth: "+str('%.2f'%depth)+"mm"
        except Exception as e:
            return e
        
    def get_depth_profile(self):  #來自realsense
        return self.depth_scale, self.d_intrin
    # def normalize(self, depth):
    #     min_val = 30
    #     max_val = 255
    #     d_med = (np.max(depth) + np.min(depth)) / 2.0
    #     d_diff = depth - d_med
    #     depth_rev = np.copy(depth)
    #     depth_rev = d_diff * (-1) + d_med
    #     depth_rev = depth_rev - np.min(depth_rev)

    #     depth_rev = cv2.normalize(depth_rev, None, min_val, max_val, cv2.NORM_MINMAX, cv2.CV_8UC1)
    #     return depth_rev
    #     #return cv2.normalize(depth, None, min_val, max_val, cv2.NORM_MINMAX, cv2.CV_8UC1)

class ArmThread(QThread):
    # control signal: working signal of robotarm
    # Input: position of bounding box
    # output: finish signal
    release_trigger = pyqtSignal(object)
    

    def __init__(self):
        super(ArmThread,self).__init__()
        self._mutex = QMutex()
        self.grip = Gripper()     #夾爪控制
        self.trans_mat = np.array([    #?
            [0.6219,-0.0021,0.],
            [-0.0028,-0.6218,0.],
            [-337.3547,-163.6015,1.0]
        ])
        self.baseline_depth = 5850  #預設高度？
        self.pose = None
        self._running = True
        self.release = False
        self.robot = RobotControl_func.RobotControl_Func() #手臂控制

    def initArmGripper(self):
        # to init arm position
        # init gripper
        self.grip.gripper_reset()

    def run(self):
        while self.running():
            if self.pose == None:
                #print("don't move")
                continue
            else:
                self.pick(self.pose)
                self.pose = None

    def testArm(self): #
        position = self.robot.get_TMPos() #[x,y,z,u,v,w]
        print("pos=",position)
        return position

    def calPosXY(self,camera_point):
        # print(camera_point)
        camera_xy = [camera_point[0],camera_point[1]]
        camera_xy.append (1.0)
        # print(camera_point)
        arm_point = np.array (np.array(camera_xy)) @ self.trans_mat
        print ("arm_point",arm_point)
        return arm_point

    def pick(self,camera_point): #?
        print (camera_point,type (camera_point))
        arm_point = self.calPosXY (camera_point)
        pos = self.cntrl.get_robot_pos ()
        # pos = self.cntrl.robot_info
        new_point = arm_point
        # print(int(new_point[0]*1000), int(new_point[1]*1000), pos[2], pos[3], pos[4], pos[5])
        self.cntrl.move_robot_pos (int (new_point[0] * 1000),int (new_point[1] * 1000),pos[2],pos[3],pos[4],pos[5],2000)
        # self.cntrl.move_robot_pos(-339197, -264430, 156320, -1668426, -24203, -74088, 1000)
        self.cntrl.wait_move_end ()
        depth_diff = (self.baseline_depth - camera_point[2])
        arm_z = 10000 + round (40 * depth_diff)
        if arm_z > 156320:
            arm_z = str (156320)
        elif arm_z < 10000:
            arm_z = str (10000)
        else:
            arm_z = str (arm_z)
        # print(10000+round(40*depth_diff))

        self.cntrl.move_robot_pos (int (new_point[0] * 1000),int (new_point[1] * 1000),arm_z,pos[3],pos[4],pos[5],2000)
        self.cntrl.wait_move_end ()
        self.grip.gripper_off()
        time.sleep(0.5)
        self.cntrl.move_robot_pos(int (new_point[0] * 1000),int (new_point[1] * 1000),pos[2],pos[3],pos[4],pos[5],2000)
        self.cntrl.wait_move_end()
        self.cntrl.move_robot_pos ('-271077','-415768','156320','-1709954','-1907','-104123',2000)
        self.cntrl.wait_move_end ()
        self.grip.gripper_on ()
        time.sleep(0.3)

        print("here")
        # go home
        self.cntrl.move_robot_pos ('2883','-246016','166040','-1709973','-1929','-104740',2000)
        self.cntrl.wait_move_end ()
        self.release = False
        print("fin")
        self.release_trigger.emit (True)

    def goHome(self): #回正
        self.robot.set_TMPos([463.1649544196957, -86.95035382249118, 402.50006103515625, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       

    def reGrasp(self):
        # print("restart streaming")
        self._mutex.lock ()
        self._running = True
        self._mutex.unlock ()
        self.run ()
    def stopGrasp(self):
        self.goHome()
        self._mutex.lock()
        self._running = False
        self._mutex.unlock()

    def running(self):
        try:
            self._mutex.lock()
            return self._running
        finally:
            self._mutex.unlock()

class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self._stream_thread = RGBDThread()
        self._stream_thread.rgbd_trigger.connect(self.updateRGBDFrame)
        self.RGBFrame.mousePressEvent = self.mouseClicked
        #self.RGBFrame.mouseReleaseEvent = self.mouseRelease
        #--------------------Bind UI with func（Start）--------------------------------
        self.CameraOn.clicked.connect(self.Start2Stop) #连接相机
        self.CameraCalibration.clicked.connect(self.CameraCalibrationF) #Camera Calibration
        self.SaveImages.clicked.connect(self.SaveImagesF)  # Save Image
        self.GripperOpen.clicked.connect(self.GripperOpenF) #Open Gripper
        self.GripperClose.clicked.connect(self.GripperCloseF)  # Close Gripper
        self.ConnectRobtoArm.clicked.connect(self.ConnectRobotArmF) #connect RobotArm
        self.SetInitPos.clicked.connect(self.SetInitPosF) #Set Init Position
        self.GetPos.clicked.connect(self.GetPosF) #Get Current Position
        self.AutoEIHCalib.clicked.connect(self.AutoEIHCalibF)
        self.DebugCalib.clicked.connect(self.DebugCalibF)
        self.TestCalibration.clicked.connect(self.TestCalibrationF)
        self.FindPlane.clicked.connect(self.FindPlaneF)
        self.PenTouchTest.clicked.connect(self.PenTouchTestF)
        self.DepthTest.clicked.connect(self.getDepthF)
        # --------------------parameter------------------------------------------------
        self.cali_img_id=1 #相機校正影像編號
        self.intrinsic = np.empty((3,3))
        self.distCoeffs = np.empty((1,5))
        self.Camera2Gripper = np.empty((4,4))
        self.depth_value=0
        # --------------------Bind UI with func（End）--------------------------------

    def getDepthF(self):  #取得深度
        self.depth_value=self._stream_thread.getDepth()
        self.OutPut.setText(str(self.depth_value))
    def FindPlaneF(self):  #??
        # self.FindPlane.setText("9")
        if(self.FindPlane.text() == "Step 1: Find Plane"):
            self.FindPlane.setText("1")
            self._arm_thread.cntrl.get_robot_pos()
            self.OutPut.setText(str(np.array(self._arm_thread.cntrl.robot_info)))
        else:
            if(self.FindPlane.text() != "9"):
                self.FindPlane.setText(str(int(self.FindPlane.text())+1))
                self._arm_thread.cntrl.get_robot_pos()
                self.OutPut.append(str(np.array(self._arm_thread.cntrl.robot_info)))
            else:
                self.FindPlane.setText("Step 1: Find Plane")
                self.OutPut.append("------------\nFinished")

    def TestCalibrationF(self): #抓取點位並且復原 ？？
        #TODO: Gripper grasp Plane
        fr = open(sys.path[0] + "/calibration/PosSet.txt", 'r+')
        dic = eval(fr.read())
        fr.close()
        GetPlanePos = StrToArray.StrToArray(dic['GetPlanePos'])
        ObjectPos = StrToArray.StrToArray(dic['CaliPos'])
        self._arm_thread.cntrl.move_robot_pos(GetPlanePos[0], GetPlanePos[1], GetPlanePos[2], GetPlanePos[3],
                                              GetPlanePos[4], GetPlanePos[5], 2000)
        self._arm_thread.cntrl.wait_move_end()
        self.GripperCloseF()
        time.sleep(1)  # time for Gripper Close
        #TODO: Make 3 move
        self._arm_thread.cntrl.move_robot_pos(ObjectPos[0][0], ObjectPos[0][1], ObjectPos[0][2], ObjectPos[0][3], ObjectPos[0][4], ObjectPos[0][5], 500)
        self._arm_thread.cntrl.wait_move_end()
        imageInFrame = self._rgb_image
        image = self.convertQImageToMat(imageInFrame)
        cv2.imshow("ETH Calibration Test",image)
        cv2.waitKey(50)
        #TODO: 不知道什么问题，但是要用waitkey刷新一下RGB Frame
        imageInFrame = self._rgb_image
        image = self.convertQImageToMat(imageInFrame)
        cv2.imshow("ETH Calibration Test", image)
        cv2.imwrite(sys.path[0] + "/calibration/EyeToHandCali/Test1.jpg", image)
        cv2.waitKey(50)

        self._arm_thread.cntrl.move_robot_pos(ObjectPos[5][0], ObjectPos[5][1], ObjectPos[5][2], ObjectPos[5][3],
                                              ObjectPos[5][4], ObjectPos[5][5], 500)
        self._arm_thread.cntrl.wait_move_end()
        imageInFrame = self._rgb_image
        image = self.convertQImageToMat(imageInFrame)
        cv2.imshow("ETH Calibration Test", image)
        cv2.waitKey(50)
        #TODO: 不知道什么问题，但是要用waitkey刷新一下RGB Frame
        imageInFrame = self._rgb_image
        image = self.convertQImageToMat(imageInFrame)
        cv2.imshow("ETH Calibration Test", image)
        cv2.imwrite(sys.path[0] + "/calibration/EyeToHandCali/Test2.jpg", image)
        cv2.waitKey(50)

        self._arm_thread.cntrl.move_robot_pos(ObjectPos[8][0], ObjectPos[8][1], ObjectPos[8][2], ObjectPos[8][3],
                                              ObjectPos[8][4], ObjectPos[8][5], 500)
        self._arm_thread.cntrl.wait_move_end()
        imageInFrame = self._rgb_image
        image = self.convertQImageToMat(imageInFrame)
        cv2.imshow("ETH Calibration Test", image)
        cv2.waitKey(50)
        #TODO: 不知道什么问题，但是要用waitkey刷新一下RGB Frame
        imageInFrame = self._rgb_image
        image = self.convertQImageToMat(imageInFrame)
        cv2.imshow("ETH Calibration Test", image)
        cv2.imwrite(sys.path[0] + "/calibration/EyeToHandCali/Test3.jpg", image)
        cv2.destroyAllWindows()
        #TODO:Calculate the result
        EIHCali.TestETHCali()
        #TODO: Put Chessboard Back
        self._arm_thread.cntrl.move_robot_pos(GetPlanePos[0], GetPlanePos[1], GetPlanePos[2], GetPlanePos[3],
                                              GetPlanePos[4], GetPlanePos[5], 2000)
        self._arm_thread.cntrl.wait_move_end()
        self.GripperOpenF()
        time.sleep(1)  # Time for gripper open
        self.SetInitPosF()

    def DebugCalibF(self): #?
        result = EIHCali.ETHCali()
        self.OutPut.setText("ETH Calibrate Finished\nT:\n" + str(result))
        EIHCali.TestETHCali()

    def AutoEIHCalibF(self): #修正手眼校正中
        shutil.rmtree(sys.path[0] + "/Saved_IMG") #清空資料夾
        try:
            if not os.path.exists(sys.path[0]+'/Saved_IMG'):
                os.makedirs(sys.path[0]+'/Saved_IMG')
            fr = open(sys.path[0] + "/EIH_PosSet.txt", 'r+')  #這時還沒有抓取joint的function所以使用position當作點位
            #print(Total_pose)
            # print(j)
            id=1
            Total_poses=[] #用來存成功的位置
            Total_imgs=[]  #用來存成功的影像
            for Pose in iter(fr):
                Pose = Pose.split(', ')
                print(Pose)
                Pose = np.array(Pose, dtype="float")
                print('Position:' + str(Pose)) 
                #點位置自己重抓
                self.Start2Stop()  #暫停影像
                self._arm_thread.robot.set_TMPos([Pose[0], Pose[1], Pose[2], Pose[3], Pose[4], Pose[5]])
                self.Start2Stop()  #恢復影像
                # print('I am in ' + str(self._arm_thread.testArm()))
                self.OutPut.setText("Begin EIH Calibration(NUM" + str(id) + ")")
                w = 11
                h = 8
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                objp = np.zeros((w * h, 3), np.float32)
                objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
                # 储存棋盘格角点的世界坐标和图像坐标对

                #在这里刷新一下Frame
                imageInFrame = self._rgb_image
                image = self.convertQImageToMat(imageInFrame)
                cv2.imshow("EIH Calibration", image)
                cv2.waitKey(50)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # 找到棋盘格角点
                ret, corners = cv2.findChessboardCorners(gray, (w, h), None)
                # 如果找到足够点对，将其存储起来
                if ret == True:
                    Total_poses.append(Pose)
                    #Total_imgs.append(image) 
                    cv2.imwrite(sys.path[0] + "/Saved_IMG/RGB" + str(id) + ".png",image)
                    print("saved:"+str(id)+"\n----------")
                    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    cv2.drawChessboardCorners(image, (w, h), corners, ret)
                    cv2.imshow("EIH Calibration", image)
                    cv2.waitKey(300)
                    id+=1
                else:
                    print("ERROR:No chessboard was found\n----------BREAK----------")
                #self.Start2Stop()  #暫停影像
                
            cv2.destroyAllWindows()
            self.SetInitPosF()
            
            #assert(len(Total_poses)>=20) #要大於20張才能繼續 用在gripper到base座標轉換
            #assert(len(Total_imgs)>=20)  #要大於20張才能繼續
            print(self.intrinsic)
            print(self.distCoeffs)
            print(sys.path[0])
            Task_r_Camera,Task_t_Camera=EIHCali.Find_Extrinsic(sys.path[0],self.intrinsic,self.distCoeffs)   #mtx  <class 'numpy.ndarray'>  ！！！！！！
            print(Total_poses)
            self.Camera2Gripper = EIHCali.EyeInHandCalibration(Total_poses,Task_r_Camera,Task_t_Camera)
            result = EIHCali.EyeInHandCalibration(sys.path[0],Total_poses)
            self.OutPut.setText("EIH Calibrate Finished\nT:\n"+str(result))
            fr.close()
        except Exception as e:
            print(str(e))

    def SetInitPosF(self):
        try:
            self.Start2Stop()  #暫停取像
            self._arm_thread.goHome()
            self.Start2Stop()  
            # fr = open(sys.path[0]+"/calibration/PosSet.txt",'r+') 
            # dic = eval(fr.read())
            # fr.close()
            # #------Use this two position for safe movement
            # # self._arm_thread.cntrl.move_robot_pos(200775,-142192,166042,-1709969,-1927,435409, 500)
            # # self._arm_thread.cntrl.wait_move_end()
            # # self._arm_thread.cntrl.move_robot_pos(153906,191943,166042,-1709969,-1927,1301238, 500)
            # # self._arm_thread.cntrl.wait_move_end()
            # #----------------------------------------------
            # ObjectPos = StrToNdArray.StrToArray(dic['InitPos'])
            # print("Go Home: "+str(ObjectPos))
            # self._arm_thread.cntrl.move_robot_pos(ObjectPos[0],ObjectPos[1],ObjectPos[2],ObjectPos[3],ObjectPos[4],ObjectPos[5],1000)
            # self._arm_thread.cntrl.wait_move_end()
            # self._arm_thread.goHome()
        except Exception as e:
            print(str(e))

    def GetPosF(self):
        try:
            self.OutPut.setText("pos="+str(self._arm_thread.testArm()))
        except Exception as e:
            print(str(e))

    def ConnectRobotArmF(self):
        try:
            if(self.ConnectRobtoArm.text() =="Connect TM_Arm"):
                self.Start2Stop()  #暫停取像
                self._arm_thread = ArmThread()
                self._arm_thread.grip.gripper_reset()
                self._arm_thread.start()
                self._arm_thread.testArm()
                # self._arm_thread.goHome()
                self._arm_thread.release_trigger.connect(self.updateRGBDFrame)
                self.ConnectRobtoArm.setText("Disconnect TM_Arm")
                self.Start2Stop() #恢復取像
            else:
                self._arm_thread.robot.pasue()
                self._arm_thread.grip.gripper_off()
                self.ConnectRobtoArm.setText("Connect TM_Arm")
        except Exception as e:
            print(str(e))

    def Start2Stop(self):
            if(self.CameraOn.text() == "Camera ON"):
                self.CameraOn.setText("Camera OFF")
                print("Camera connected.")
                if(self._stream_thread._running == True):
                    self._stream_thread.start()
                else:
                    self._stream_thread.restart()
            else:
                self.CameraOn.setText("Camera ON")
                print("Camera disconnected.")
                self._stream_thread.pause()

    def updateRGBDFrame(self, rgbd_image):
        QApplication.processEvents()
        self._rgb_image = QImage(rgbd_image[0][:], rgbd_image[0].shape[1], rgbd_image[0].shape[0], QImage.Format_RGB888)
        #self._d_image = QImage(rgbd_image[2][:], rgbd_image[2].shape[1], rgbd_image[2].shape[0], rgbd_image[2].shape[1]*1, QImage.Format_RGB888)
        self._d_image = QImage(rgbd_image[1][:], rgbd_image[1].shape[1], rgbd_image[1].shape[0], QImage.Format_Grayscale8)
        #qimage = QImage(depth_map_normalized.data, width, height, QImage.Format_Grayscale8)
        #bytesPerline = channel * width
        try:
            #TODO: one point is too small, 7*7 all red
            
            if mode == 0: # Draw a point
                for i in range(-3, 3, 1):
                    for j in range(-3, 3, 1):
                        self._rgb_image.setPixel(realX + i,realY + j, qRgb(255, 0, 0))  #在原始未縮放的圖片上畫圖
                        #self._d_image.setPixel(realX + i, realY + j, qRgb(255, 0, 0))
            # if mode == 1: # Draw a BBox  但應該用不到
            #     if realX > releaseX:
            #         x1 = releaseX
            #         x2 = realX
            #     else:
            #         x2 = releaseX
            #         x1 = realX
            #     if realY > releaseY:
            #         y1 = releaseY
            #         y2 = realY
            #     else:
            #         y2 = releaseY
            #         y1 = realY
            #     for i in range(-2, 2, 1):
            #         for j in range(-2, 2, 1):
            #             for w in range(0, x2 - x1, 1):
            #                 self._rgb_image.setPixel(x1 + w + i, y1 + j, qRgb(255, 0, 0))
            #                 self._rgb_image.setPixel(x1 + w + i, y2 + j, qRgb(255, 0, 0))
            #                 self._d_image.setPixel(x1 + w + i, y1 + j, qRgb(255, 0, 0))
            #                 self._d_image.setPixel(x1 + w + i, y2 + j, qRgb(255, 0, 0))
            #             for h in range(0, y2 - y1, 1):
            #                 self._rgb_image.setPixel(x1 + i, y1 + h + j, qRgb(255, 0, 0))
            #                 self._rgb_image.setPixel(x2 + i, y1 + h + j, qRgb(255, 0, 0))
            #                 self._d_image.setPixel(x1 + i, y1 + h + j, qRgb(255, 0, 0))
            #                 self._d_image.setPixel(x2 + i, y1 + h + j, qRgb(255, 0, 0))
        except:
            pass
        
        self.RGBFrame.setPixmap(QPixmap.fromImage(self._rgb_image).scaled(890,500))
        self.DepthFrame.setPixmap(QPixmap.fromImage(self._d_image).scaled(409,230))
        # self.RGBFrame.setPixmap(QPixmap.fromImage(self._rgb_image).scaled(890,500,Qt.KeepAspectRatio))
        # self.DepthFrame.setPixmap(QPixmap.fromImage(self._d_image).scaled(409,230,Qt.KeepAspectRatio))
        

    def mouseClicked(self,event):  
        global realX, realY, mode,imgX,imgY
        mode = 0
        # realX = int(event.pos().x() / 889 * 640 )  #889,500是視窗大小
        # realY = int(event.pos().y() / 500 * 480 )
        realX = int(event.pos().x() * 640/890 )  #890,500是視窗大小
        realY = int(event.pos().y() * 480/500 )
        imgX = event.pos().x()
        imgY = event.pos().y()
        print("-------Click-------\n",event.pos().x())
        print(event.pos().y())
        print("-------real-------\n",realX)
        print(realY)
        mode = 0
        self.getDepthF()

    # def mouseRelease(self,event):
    #     global releaseX , releaseY, mode
    #     releaseX = int(event.pos().x() * 640/889 )
    #     releaseY = int(event.pos().y() * 480/500 )
    #     if releaseX == realX and releaseY == realY:
    #         mode = 0
    #         self.getDepthF()
    #         # A point for estimateCoordinate
    #     else:
    #         mode = 1
    #         # A BBox for depthCalculate
    #         self.getDepthF()
        # print("-------Release-------\n",event.pos().x())
        # print(event.pos().y())

    def PenTouchTestF(self): #下爪點位測試
        try:
            Estimate_Coord= EIHCali.EstimateCoord(realX,realY,self.intrinsic,self.depth_value)
            print(Estimate_Coord)
            # self._arm_thread.cntrl.move_robot_pos(str(int(EstimateCoordResult[0])), str(int(EstimateCoordResult[1])), str(int(EstimateCoordResult[2])), -1710090 ,-24420 ,1620220 , 500)
            # self._arm_thread.cntrl.wait_move_end()
        except:
            print("No point select!")

    def convertQImageToMat(self,incomingImage):
        incomingImage = incomingImage.convertToFormat(4)
        width = incomingImage.width()
        height = incomingImage.height()
        ptr = incomingImage.bits()
        ptr.setsize(incomingImage.byteCount())
        arr = np.array(ptr).reshape(height,width,4)  # Copies the data
        return arr

    def CameraCalibrationF(self):
        self.Start2Stop() #用這個去控制thread
        #self.CameraCalibration.setText("20") #调试时直接用拍摄好的照片做校正
   
        self.intrinsic,self.distCoeffs=EIHCali.CameraCalibration(sys.path[0])
        #to do np.save()
        
        # np.save(sys.path[0]+'/INS.npy',np.asarray(mtx))
        # np.save(sys.path[0]+'/dist.npy',np.asarray(dist))
        dict = {'intrinsic': str(self.intrinsic), 'distCoeffs': str(self.distCoeffs)}
        fw = open(sys.path[0]+"/CameraCali.txt",'w+')
        fw.write(str(dict))
        fw.close()
            #fr = open(sys.path[0]+"/calibration/CameraCali.txt",'r+')
            #dic = eval(fr.read())
            #print(dic['MTX'])
            #fr.close()
        self.OutPut.append(dict['intrinsic'])
        self.OutPut.setText("Camera Calibrate Finished")
        cv2.destroyAllWindows() 
        self.Start2Stop()

    def SaveImagesF(self):
        try:
            if not os.path.exists(sys.path[0]+'/Saved_IMG'):
                os.makedirs(sys.path[0]+'/Saved_IMG')
            imageInFrame = self._rgb_image
            DepthImageInFrame = self._d_image
            image = self.convertQImageToMat(imageInFrame)
            depth_image = self.convertQImageToMat(DepthImageInFrame)
            #ticks = time.time()
            # cv2.imwrite(sys.path[0]+"/saved/RGB" + str(ticks) + ".jpg",image)
            # cv2.imwrite(sys.path[0]+"/saved/D" + str(ticks) + ".jpg",depth_image)
            cv2.imwrite(sys.path[0]+"/Saved_IMG/RGB" + str(self.cali_img_id) + ".png",image)
            #cv2.imwrite(sys.path[0]+"/Saved_IMG/D" + str(self.cali_img_id) + ".png",depth_image) #深度圖使用
            self.OutPut.setText("Saved:"+str(self.cali_img_id))
            self.cali_img_id +=1
        except Exception as e:
            self.OutPut.setText(str(e))

    def GripperOpenF(self):
        try:
            self._stream_thread.pause() #暫停取像
            print("Open Gripper")
            self._arm_thread.grip.gripper_on()
            self._stream_thread.restart() #恢復取像
        except Exception as e:
            self.OutPut.setText(str(e))

    def GripperCloseF(self):
        try:
            self._stream_thread.pause() #暫停取像
            print("Close Gripper")
            self._arm_thread.grip.gripper_off()
            self._stream_thread.restart() #恢復取像
        except Exception as e:
            self.OutPut.setText(str(e))

    
if __name__ == "__main__":
    rospy.init_node("RobotControl",anonymous=True)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
