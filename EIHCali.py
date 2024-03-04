import numpy as np
import cv2
import sys
import glob
import math

def RotationTrans(v):
    #目前是使用弧度
    # print("----------VVVVVV------")
    # print("v:",v)
    #pi = np.pi / 180 #轉成弧度
    # tmp_v = v[0]
    # v[0] = v[2]
    # v[2] = tmp_v
    # pi =   1
    r1_mat = np.zeros((3, 3), np.float32)
    r2_mat = np.zeros((3, 3), np.float32)
    r3_mat = np.zeros((3, 3), np.float32)

    r = np.zeros((3, 1), np.float32)
    r[0] = 0
    r[1] = 0
    #r[2] = float(v[2]) * pi # 如果是角度轉成弧度
    r[2] = float(v[2]) 
    r3_mat, jacobian = cv2.Rodrigues(r)
    # print("r3_mat:",r3_mat)
    r[0] = 0
    r[1] = float(v[1])
    r[2] = 0
    # print('ys ', math.sin(v[1]))
    # print('yc ', math.cos(v[1]))
    r2_mat, jacobian = cv2.Rodrigues(r)
    # print("r2_mat:",r2_mat)
    r[0] = float(v[0])
    r[1] = 0
    r[2] = 0
    r1_mat, jacobian = cv2.Rodrigues(r)
    # print("r1_mat:",r1_mat)

    result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
    # print(v)
    # v = [element / 180 * np.pi for element in v]
    # print(v)
    # rxs = math.sin(v[0])
    # rxc = math.cos(v[0])
    # rys = math.sin(v[1])
    # ryc = math.cos(v[1])
    # rzs = math.sin(v[2])
    # rzc = math.cos(v[2])

    # r1_mat = np.array([[1,0,0],[0, rxc, -rxs], [0, rxs, rxc]])
    # print(r1_mat)
    # r2_mat = np.array([[ryc,0,rys],[0, 1, 0], [-rys, 0, ryc]])
    # print(r2_mat)
    # r3_mat = np.array([[rzc,-rzs,0],[rzs, rzc, 0], [0, 0, 1]])
    # print(r3_mat)
    
    

    # result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
    
    # print('result')
    print(result)
    return result

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    # print([x,y,z])
    return np.array([x, y, z])

def CameraCalibration(path): #相機校正 只用來求內參跟失真係數
    w= 11
    h = 8
    square_size=20 #20mm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
    objp = np.zeros((w * h,3),np.float32)
    objp[:,:2] = np.mgrid[0:w,0:h].T.reshape (-1,2)*square_size
    # 储存棋盘格角点的世界坐标和图像坐标对
    objpoints = []  # 在世界坐标系中的三维点
    imgpoints = []  # 在图像平面的二维点
    #self.CameraCalibration.setText("Camera Calibration")
    #20张图片收集完成，开始做Camera Calibration把算出来的结果储存到/calibration/CameraCali/CameraCali.txt
    images = sorted(glob.glob(path +'/CameraCali/*.png'))
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (w, h), None)
        if ret == True:
            subPix_corners=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            objpoints.append(objp)
            imgpoints.append(subPix_corners)
            # cv2.drawChessboardCorners(img,(w,h),subPix_corners,ret)
            # cv2.imshow("Camera Calibration",img)
            # cv2.waitKey(100)
    ret, mtx, dist, _ , _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return mtx,dist

def Find_Extrinsic(path,intrinsic,distCoeffs): #尋找單相機外參(也就是物體(task)世界座標到相機座標的轉換關係,但需要有已知的內參跟失真參數
    w = 11
    h = 8
    square_size=20 #20mm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
    objp = np.zeros((w * h,3),np.float32)
    objp[:,:2] = np.mgrid[0:w,0:h].T.reshape (-1,2)*square_size
    # 储存棋盘格角点的世界坐标和图像坐标对
    #self.CameraCalibration.setText("Camera Calibration")
    #30张图片收集完成，开始做Camera Calibration把算出来的结果储存到/calibration/CameraCali/CameraCali.txt
    files = glob.glob(path +'/Saved_IMG/*.png') #不加入sort 會變成1 -> 10 ->2 順序怪怪的變成已開頭第一個為主
    images = sorted(files, key=lambda x: int(x.split('/')[-1].split('.')[0]))
    Task_r_Camera = []  #如果有30張影像就會有20組外參
    Task_t_Camera = []

    objpoints = []
    imgpoints = []
    for frame in images:
        print(frame)
        img = cv2.imread(frame)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (w, h), None)
        if ret == True:
            subPix_corners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            # cv2.drawChessboardCorners(img,(w,h),subPix_corners,ret)
            # cv2.imshow("ext",img)
            # cv2.waitKey(100)
            _, rvec, tvec = cv2.solvePnP(objp, subPix_corners, intrinsic, distCoeffs)
            print(rvec)
            print(tvec)
            r_mat,_ = cv2.Rodrigues(rvec)
            Task_r_Camera.append(r_mat)

            Task_t_Camera.append(tvec)
            # imgpoints.append(subPix_corners)
            # objpoints.append(objp)

    #objpoints = [x * 20 for x in objpoints] #20mm 代表棋盤格大小
    #_, rvec, tvec = cv2.solvePnP(objp, corners, intrinsic, distCoeffs)
    # ret, mtx, dist, rvec, tvec = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    # #mtx[0][0] = 642.838
    # #mtx[1][1] = 637.720

    # print(Task_r_Camera[19])
    # print(Task_t_Camera[19])
    # print("----------")
    # print(mtx)
    # #print("rvec: ")
    # rmatxs = []
    # for r in rvec:
    #     rmatx = 0
    #     rmatx = cv2.Rodrigues(r,rmatx)
    #     rmatxs.append(rmatx[0])

    # print(rmatxs[19])
    # print(tvec[19])
    # print("sdddSdsd")

    return Task_r_Camera,Task_t_Camera   #每一個的rotation跟translation合併就是 task到camera的外參了

def EyeInHandCalibration(Total_poses,Task_r_Camera,Task_t_Camera):  #手眼校正  還沒檢查
    Gripper_r_Base = []
    Gripper_t_Base = []
    import math

    for Pose in Total_poses:  #記住外參有幾組Gripper2Base 的 R,T就有幾組
        [x,y,z,u,v,w]=Pose
        G2B_translation = np.matrix([x,y,z]).T
        Gripper_t_Base.append(G2B_translation)
        print([x,y,z,u,v,w])

        #G2B_rotation_vect = np.asarray([u,v,w])
        #把尤拉角轉成 axis with angle magnitude
        G2B_rotation_mat= RotationTrans([u,v,w])
        Gripper_r_Base.append(G2B_rotation_mat)
        

    Camera_r_Gripper, Camera_t_Gripper = cv2.calibrateHandEye(Gripper_r_Base,Gripper_t_Base,Task_r_Camera,Task_t_Camera,method=cv2.CALIB_HAND_EYE_TSAI)

    Camera2Gripper = np.r_[np.c_[Camera_r_Gripper, Camera_t_Gripper], [[0, 0, 0, 1]]]
    print('CALIB_HAND_EYE_TSAI')
    print(Camera2Gripper)
    print("共"+str(len(Total_poses))+"組pose")

    return Camera2Gripper


def EstimateCoord(realX,realY,intrinsic,camera_depth,Camera2Gripper,Current_pos): 

    [x,y,z,u,v,w] = Current_pos  #這隻手臂讀出來的u v w是角度 ,cv2.Rodrigues只吃弧度  #法蘭面向下15公分約在開著的夾爪正中間
    G2B_translation = np.matrix([x,y,z]).T
    print(G2B_translation)
    # G2B_translation = np.matrix([39.65148707,-1.8229379,29.29505309]).T
    # G2B_rotation_vect = np.asarray([u , v, w]) #轉成弧度
    # G2B_rotation_mat,_ = cv2.Rodrigues(G2B_rotation_vect)
    #把尤拉角轉成 axis with angle magnitude
    G2B_rotation_mat= RotationTrans([u,v,w])
    # rmatx = 0
    # R = np.matrix([u,v,w])
    # rmatx = cv2.Rodrigues(R,rmatx)[0]
    # print("test")
    # print(rmatx)
    # print(G2B_rotation_mat)
    # print("-------test")
    # G2B_rotation_mat=np.asarray(
    # [[ 0.99888145, -0.04660927,  0.00796429],
    # [ 0.04537618,  0.99223829,  0.11577645],
    # [-0.01329873 ,-0.11528555,  0.99324337]])
    Gripper2Base = np.r_[np.c_[G2B_rotation_mat, G2B_translation], [[0, 0, 0, 1]]]
    
    #Camera2Base = Camera2Gripper * Gripper2Base
    print(Gripper2Base)

    cx = intrinsic[0][2]
    fx = intrinsic[0][0]
    cy = intrinsic[1][2]
    fy = intrinsic[1][1]

    point = np.zeros((4, 1), dtype=np.float64)

    point[0, 0]  = ((realX - cx) / fx ) * camera_depth
		
    point[1, 0]  = ((realY - cy) / fy ) * camera_depth

    point[2, 0] = camera_depth

    point[3, 0] = 1.0
    print("相機座標")  #相機座標
    print(point)  #相機座標
    print(Camera2Gripper)
    print(Camera2Gripper @ point)
    print("camera到gripper")
    print(Gripper2Base @ Camera2Gripper)
    Base_point = Gripper2Base @ Camera2Gripper @ point      #從右邊往左看,相機座標到夾爪座標再到base座標
    print("_____")
    return Base_point


# EstimateCoord(194,122) #707， -503
# EstimateCoord(496,110) #958， -494
# EstimateCoord(302,301) #799， -664，
# EstimateCoord(411,263) #888， -625， 536
# EstimateCoord(612, 414)  # 958， -494
 
if __name__ == "__main__":
    mtx,dis = CameraCalibration('/media/eric/新增磁碟區/calibration')
    # print(mtx)
    # print(type(mtx))
    # print(dis)
    Task_r_Camera,Task_t_Camera = Find_Extrinsic('/media/eric/新增磁碟區/calibration',mtx,dis)  #測試中 怪怪的t_v
    # print(len(total_r_mat))
    # print(len(total_tvect))
    # print(total_r_mat)
    # print(total_tvect)
    total_pose=np.array(
        [[594.0838069345818, -138.5408801148759, 467.4356994628906, -173.66317749023438, 1.6348413228988647, 40.24143981933594],
        [651.1750404833966, -137.92859227889605, 377.5372314453125, -161.86044311523438, 6.094341278076173, 43.08310317993164],
        [380.57412494766294, -154.7507657518352, 465.6792907714844, 155.37606811523438, 0.7027701139450073, 40.455318450927734],
        [552.544283046787, 91.85597185453206, 469.34497070312506, -178.34031677246094, 26.35577392578125, 79.72480773925783],
        [505.931211898094, 40.430569496767646, 417.74749755859375, -178.93786621093753, 24.97023963928223, 77.29475402832031],
        [379.65062208488035, -171.83297921094038, 396.77734375, 149.84552001953128, -2.5344710350036626, 37.02196502685547],
        [313.1101271400841, -216.13275617872853, 464.00189208984375, 165.38563537597656, -8.392745971679688, 7.755660057067872],
        [316.2141420548246, -218.0021395206294, 411.0020446777344, 164.49418640136716, -22.723831176757812, 1.4787429571151733],
        [233.0353363977323, -175.30989595431373, 683.2503662109375, 171.5413818359375, -21.719711303710938, -0.6140484809875488],
        [414.79957780511876, -237.51933219296103, 603.5322265625, -174.9716339111328, -15.467803955078123, -3.737208127975464],
        [503.0539726536259, -51.29590565995254, 375.5477600097656, 175.12493896484375, 14.943559646606447, 39.2091178894043],
        [580.6029007416711, 9.051993968462197, 464.00018310546875, -171.6441650390625, 15.597503662109375, 41.904556274414055],
        [704.5625735263708, -5.463094734469443, 276.04052734375, -159.20770263671875, 31.903396606445312, 51.70433044433594],
        [389.1585950523619, -52.71503927683339, 741.1685791015625, 164.48062133789062, -0.1004832461476326, 38.60519790649414],
        [699.1994767418531, -34.262051157654, 542.7216186523438, -159.6223907470703, 16.441404342651367, 34.38968276977538],
        [315.4986517597071, -180.57179984143065, 504.970458984375, 146.3219757080078, -0.15390565991401672, 25.624868392944336],
        [546.869690706043, 155.49661796227036, 508.43298339843756, -174.7596893310547, 31.34952735900879, 69.66913604736328],
        [538.1611726481076, 146.33208853653616, 424.66839599609375, -173.2864227294922, 31.832077026367184, 70.44954681396486],
        [162.116003730616, -245.29825936014004, 503.5932006835938, 152.44273376464844, -21.98662567138672, -4.411279201507568],
        [233.795304093496, -271.0178254177077, 411.585693359375, 162.21205139160156, -28.868242263793945, -9.436110496520996]])
    
    Camera2Gripper=EyeInHandCalibration(total_pose,Task_r_Camera,Task_t_Camera)
    #pos=[463.1625591300011, -86.95072066866062, 402.4959716796875, -176.01728820800784, 0.5390524864196777, 46.80111312866211]
    
    import rospy
    import RobotControl_func_ros1 as RobotControl_func
    rospy.init_node("RobotControl2",anonymous=True)
    robot = RobotControl_func.RobotControl_Func()

    #robot.set_TMPos([594.0838069345818, -138.5408801148759, 467.4356994628906, -173.66317749023438, 1.6348413228988647, 40.24143981933594])        
    pos = robot.get_TMPos()
    point = EstimateCoord(412,228 , mtx , 619, Camera2Gripper,pos)  #自己點的點位  注意相機上下相反
    robot.set_TMPos([368.5520935058594, -444.62713623046875, 390.8355712890625, -3.13378453158979, -0.0104040954889096, 0.7755800871437349])       

    # x,y 相機座標要相反測試
    robot.set_TMPos([368.5520935058594+132.53765527, -444.62713623046875+137.74397411, 390.8355712890625, -3.13378453158979, -0.0104040954889096, 0.7755800871437349])       

    #robot.set_TMPos([after_xyz[0],after_xyz[1], 100, 173.10702514648438, -0.027281902730464935, 46.70990753173829])       
    #robot.set_TMPos([463.1649544196957-34.60012977,-86.95035382249118-75.7420336 , 402.50006103515625, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
    #robot.set_TMPos([513.2411925,83.63995229, 100, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
    #robot.set_TMPos([487.10260837,89.04113697,100, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       

    #robot.set_TMPos([88.61430884,-123.83022553, 59.74281404, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
    #robot.set_TMPos([275.78520487 ,200, 100, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
