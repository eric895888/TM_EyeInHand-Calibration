import numpy as np
import cv2
import sys
import glob
import math

# def eye_in_hand_calibration(robot_arm,
#     chessboard_intercorner_shape,
#     intrinsic_matrix,
#     distortion_coefficients,
#     method=cv2.CALIB_HAND_EYE_TSAI):
    

#     StartNum=1 #number of images and poses
#     EndNum=20 #which will not be calculate(stop at N-1)
#     w = 11
#     h = 8
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, sys.float_info.epsilon)
#     objp = np.zeros((w * h, 3), np.float32)
#     objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)  
#     # print("OBJP", objp)
#     # 储存棋盘格角点的世界坐标和图像坐标对
#     objpoints = []  # 在世界坐标系中的三维点
#     imgpoints = []  # 在图像平面的二维点
#     for i in range(StartNum, EndNum):
#         img = cv2.imread("C:/Users/12990/Desktop/EIH/20210429_EIH_Cali/" + str(i) + ".jpg")
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         # 找到棋盘格角点
#         ret, corners = cv2.findChessboardCorners(gray, (w, h),flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS)
#         # 如果找到足够点对，将其存储起来
#         if ret == True:
#             print(i)
#             cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
#             objpoints.append(objp)
#             imgpoints.append(corners)
#             # 将角点在图像上显示
#             cv2.drawChessboardCorners(img, (w, h), corners, ret)
#             cv2.namedWindow("EIH Calibration", cv2.WINDOW_NORMAL)
#             cv2.resizeWindow('EIH Calibration', (860, 540))
#             cv2.imshow("EIH Calibration",img)
#             cv2.waitKey(0)
#         else:
#             print("ERROR!!: " + str(i) + ": no chessboard ")
#     cv2.destroyAllWindows()
#     # Get Rotation and Translation from Camera To Plane
#     objpoints = [x * 15 for x in objpoints]

#     for index, joint_config in enumerate(calibration_joint_config_list):
#         logging.info("Move to joint config: {}".format(joint_config))
#         robot_arm.move_to_joint_config(joint_config, speed)
#         calibration_record["pose_list"].append(robot_arm.get_cart_pose())

#         chessboard_image = IMAGE() # TODO cache image
#         image_path = IMAGE_SAVE_FOLDER_PATH + f"/chessboard_{index}.jpg"
#         chessboard_image.save(image_path)
#         calibration_record["image_paths"].append(image_path)

#     obj_points, img_points, gray_shape = corner_detection(
#         calibration_record["image_paths"],
#         chessboard_intercorner_shape,
#         show_result=False,
#     )
#     # caculate extrinsic matrix
#     ret, rvecs, tvecs = cv2.solvePnP(
#         obj_points, img_points, intrinsic_matrix, distortion_coefficients
#     )

#     # eye in hand calibration
#     R_gripper2base = calibration_record["pose_list"][:, 3:]  # / 1000.0
#     t_gripper2base = calibration_record["pose_list"][:, 0:3]  # / 1000.0
#     R_target2cam = rvecs
#     t_target2cam = tvecs

#     R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
#         R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method
#     )

#     result = {
#         "R_cam2gripper": R_cam2gripper.tolist(),
#         "t_cam2gripper": t_cam2gripper.tolist(),
#     }

#     with open(f"{IMAGE_SAVE_FOLDER_PATH}/cam2gripper.json", "w") as f:
#         json.dump(result, f)

#     logging.info("Power off")
#     robot_arm.power_off()

#     return result

# def RotationTrans(v):
#     # print("----------VVVVVV------")
#     # print("v:",v)
#     pi = np.pi / 180
#     # tmp_v = v[0]
#     # v[0] = v[2]
#     # v[2] = tmp_v
#     # pi =   1
#     r1_mat = np.zeros((3, 3), np.float32)
#     r2_mat = np.zeros((3, 3), np.float32)
#     r3_mat = np.zeros((3, 3), np.float32)

#     r = np.zeros((3, 1), np.float32)
#     r[0] = 0
#     r[1] = 0
#     r[2] = float(v[2]) * pi
#     r3_mat, jacobian = cv2.Rodrigues(r)
#     # print("r3_mat:",r3_mat)
#     r[0] = 0
#     r[1] = float(v[1]) * pi
#     r[2] = 0
#     # print('ys ', math.sin(v[1]))
#     # print('yc ', math.cos(v[1]))
#     r2_mat, jacobian = cv2.Rodrigues(r)
#     # print("r2_mat:",r2_mat)
#     r[0] = float(v[0]) * pi
#     r[1] = 0
#     r[2] = 0
#     r1_mat, jacobian = cv2.Rodrigues(r)
#     # print("r1_mat:",r1_mat)

#     result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
#     # print(v)
#     # v = [element / 180 * np.pi for element in v]
#     # print(v)
#     # rxs = math.sin(v[0])
#     # rxc = math.cos(v[0])
#     # rys = math.sin(v[1])
#     # ryc = math.cos(v[1])
#     # rzs = math.sin(v[2])
#     # rzc = math.cos(v[2])

#     # r1_mat = np.array([[1,0,0],[0, rxc, -rxs], [0, rxs, rxc]])
#     # print(r1_mat)
#     # r2_mat = np.array([[ryc,0,rys],[0, 1, 0], [-rys, 0, ryc]])
#     # print(r2_mat)
#     # r3_mat = np.array([[rzc,-rzs,0],[rzs, rzc, 0], [0, 0, 1]])
#     # print(r3_mat)
    
    

#     # result = np.dot(np.dot(r3_mat, r2_mat), r1_mat)
    
#     # print('result')
#     print(result)
#     return result


# def isRotationMatrix(R):
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype=R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6


# def rotationMatrixToEulerAngles(R):
#     assert (isRotationMatrix(R))

#     sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
#     singular = sy < 1e-6
#     if not singular:
#         x = math.atan2(R[2, 1], R[2, 2])
#         y = math.atan2(-R[2, 0], sy)
#         z = math.atan2(R[1, 0], R[0, 0])
#     else:
#         x = math.atan2(-R[1, 2], R[1, 1])
#         y = math.atan2(-R[2, 0], sy)
#         z = 0
#     # print([x,y,z])
#     return np.array([x, y, z])

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
    images = glob.glob(path +'/CameraCali/*.png')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (w, h), None)
        if ret == True:
            subPix_corners=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            objpoints.append(objp)
            imgpoints.append(subPix_corners)
            cv2.drawChessboardCorners(img,(w,h),subPix_corners,ret)
            cv2.imshow("Camera Calibration",img)
            cv2.waitKey(100)
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
    images = glob.glob(path +'/Saved_IMG/*.png')
    Task_r_Camera = []  #如果有30張影像就會有20組外參
    Task_t_Camera = []

    objpoints = []
    imgpoints = []
    for frame in images:
        img = cv2.imread(frame)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (w, h), None)
        if ret == True:
            subPix_corners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            cv2.drawChessboardCorners(img,(w,h),subPix_corners,ret)
            cv2.imshow("ext",img)
            cv2.waitKey(100)
            _, rvec, tvec = cv2.solvePnP(objp, subPix_corners, intrinsic, distCoeffs)
            print(rvec)
            r_mat,_ = cv2.Rodrigues(rvec)
            Task_r_Camera.append(r_mat)

            Task_t_Camera.append(tvec)
            imgpoints.append(subPix_corners)
            objpoints.append(objp)

    #objpoints = [x * 20 for x in objpoints] #20mm 代表棋盤格大小
    #_, rvec, tvec = cv2.solvePnP(objp, corners, intrinsic, distCoeffs)
    ret, mtx, dist, rvec, tvec = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    #mtx[0][0] = 642.838
    #mtx[1][1] = 637.720

    print(Task_r_Camera[29])
    print(Task_t_Camera[29])
    print("----------")
    print(mtx)
    #print("rvec: ")
    rmatxs = []
    for r in rvec:
        rmatx = 0
        rmatx = cv2.Rodrigues(r,rmatx)
        rmatxs.append(rmatx[0])

    print(rmatxs[29])
    print(tvec[29])
    print("sdddSdsd")

    return Task_r_Camera,Task_t_Camera   #每一個的rotation跟translation合併就是 task到camera的外參了

def EyeInHandCalibration(Total_poses,Task_r_Camera,Task_t_Camera):  #手眼校正  還沒檢查
    Gripper_r_Base = []
    Gripper_t_Base = []
    
    for Pose in Total_poses:  #記住外參有幾組Gripper2Base 的 R,T就有幾組
        [x,y,z,u,v,w]=Pose
        G2B_translation = np.matrix([x,y,z]).T
        Gripper_t_Base.append(G2B_translation)
        print([x,y,z,u,v,w])

        G2B_rotation_vect = np.matrix([u,v,w])
        G2B_rotation_mat,_ = cv2.Rodrigues(G2B_rotation_vect)
        Gripper_r_Base.append(G2B_rotation_mat)
        

    Camera_r_Gripper, Camera_t_Gripper = cv2.calibrateHandEye(Gripper_r_Base,Gripper_t_Base,Task_r_Camera,Task_t_Camera,method=cv2.CALIB_HAND_EYE_TSAI)

    Camera2Gripper = np.r_[np.c_[Camera_r_Gripper, Camera_t_Gripper], [[0, 0, 0, 1]]]
    print('CALIB_HAND_EYE_TSAI')
    print(Camera2Gripper)


    return Camera2Gripper


def EstimateCoord(realX,realY,intrinsic,camera_depth,Camera2Gripper,Current_pos): 

    [x,y,z,u,v,w] = Current_pos  #這隻手臂讀出來的u v w是角度 ,cv2.Rodrigues只吃弧度  #法蘭面向下15公分約在開著的夾爪正中間
    G2B_translation = np.matrix([x,y,z]).T
    print(G2B_translation)
    # G2B_translation = np.matrix([39.65148707,-1.8229379,29.29505309]).T
    G2B_rotation_vect = np.matrix([u* np.pi / 180 , v* np.pi / 180 , w* np.pi / 180]) #轉成弧度
    G2B_rotation_mat,_ = cv2.Rodrigues(G2B_rotation_vect)
    
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
    print("測試夾爪座標")
    Base_point = Gripper2Base @ Camera2Gripper @ point      #從右邊往左看,相機座標到夾爪座標再到base座標
    print("_____")
    return Base_point


# EstimateCoord(194,122) #707， -503
# EstimateCoord(496,110) #958， -494
# EstimateCoord(302,301) #799， -664，
# EstimateCoord(411,263) #888， -625， 536
# EstimateCoord(612, 414)  # 958， -494
 
if __name__ == "__main__":
    mtx,dis = CameraCalibration('/home/eric/catkin_ws/src/vgn/scripts/calibration')
    # print(mtx)
    # print(type(mtx))
    # print(dis)
    Task_r_Camera,Task_t_Camera = Find_Extrinsic('/home/eric/catkin_ws/src/vgn/scripts/calibration',mtx,dis)  #測試中 怪怪的t_v
    # print(len(total_r_mat))
    # print(len(total_tvect))
    # print(total_r_mat)
    # print(total_tvect)
    total_pose=np.array(
        [[520.0401803122703, -34.401193751803454, 436.8352355957031, -177.34342956542966, 11.693657875061035, 56.585323333740234],
        [430.39916089125046, -69.40250467843593, 519.808349609375, 170.21800231933594, 9.781499862670898, 53.7475471496582],
        [460.3151315315966, -116.84994586033118, 400.67767333984375, 179.58535766601562, -1.582494854927063, 40.881065368652344],
        [575.0803553485663, -108.46437111702744, 451.9459533691406, -163.1959991455078, 1.006693959236145, 40.77115249633789],
        [564.4543755213471, -172.14735558855622, 398.3766174316407, -167.48153686523438, -10.199124336242678, 27.746347427368164],
        [470.92575776766546, -174.4907041813721, 383.2444152832031, 178.42539978027344, -9.305204391479492, 30.36167716979981],
        [457.92585164044834, -201.0385846129479, 356.17559814453125, -177.20016479492188, -12.158486366271973, 24.604085922241214],
        [500.75220715307523, -67.31991897453177, 436.168701171875, -171.5462188720703, -0.5468732714653016, 45.981658935546875],
        [608.2147213813653, -73.66107080514618, 364.29254150390625, -151.33677673339844, 2.263035297393799, 43.778190612792976],
        [649.6171095393494, -68.55759321265903, 270.20684814453125, -150.0587615966797, 2.4488115310668945, 43.83047485351563],
        [643.4999280847705, -69.29456558787517, 194.40777587890625, -142.82818603515625, 3.474827289581299, 44.2051773071289],
        [635.657965416609, 42.134656270556725, 253.8122100830078, -132.541015625, 10.704540252685549, 64.9795913696289],
        [668.6335302164138, -249.03380265296778, 316.387451171875, -142.95672607421875, -9.469378471374512, 14.259588241577148],
        [433.8845771366819, -165.87848324291448, 334.4363098144532, 175.25450134277344, -21.257265090942383, 26.84083366394043],
        [390.20541217604193, -279.4406350472095, 360.50189208984375, -172.87237548828125, -29.726171493530273, 2.6375067234039307],
        [376.1134373711851, -265.46749682206445, 304.103759765625, -177.33058166503906, -32.74168395996094, 4.478131294250488],
        [453.34016662653084, 77.29513527635281, 356.8872985839844, -163.47698974609378, 21.885900497436527, 84.49977111816406],
        [473.8777688931397, 82.9236344748662, 417.61126708984375, -178.64524841308594, 22.26116180419922, 50.29847717285157],
        [406.2929654050867, 150.16925631049338, 367.2850646972656, -166.40925598144534, 38.33643341064453, 80.37854766845703],
        [506.51046199963787, 74.98564442705751, 496.83599853515625, 176.73255920410156, 15.58269786834717, 43.300201416015625],
        [377.2567134582413, -79.9652466997998, 553.9547119140625, 167.97940063476562, -3.989835023880005, 24.01165771484375],
        [392.3728688202051, -64.29230516937348, 372.83428955078125, 163.4627838134766, -10.299189567565916, 25.309354782104496],
        [561.890034480224, -60.61947368973171, 435.5775146484375, -168.67794799804688, -1.9323612451553345, 14.842402458190916],
        [431.6793784954395, -75.06780717941864, 570.2179565429688, 177.58749389648438, -11.761266708374023, 16.49750900268555],
        [359.8569514811139, -116.63332319727752, 350.3479614257813, 165.86915588378906, -10.362303733825684, 16.955942153930664],
        [435.7101547301146, -112.02427871404142, 465.2738037109375, -178.356689453125, 1.8564635515213013, 15.647821426391602],
        [406.489454687194, -101.04355864097492, 473.68066406250006, 173.24722290039062, -8.189007759094238, 16.352174758911133],
        [398.4513264247217, -110.03112794832775, 355.9123229980469, 169.6910400390625, -8.38566780090332, 16.482513427734375],
        [638.3835245596073, -83.48575070237877, 505.6443481445313, -169.22593688964844, 6.634911060333253, 16.138320922851562],
        [485.8202571213391, -305.34384267875924, 496.5609130859375, -157.02288818359375, -7.420004367828369, -19.40464210510254]])
    
    Camera2Gripper=EyeInHandCalibration(total_pose,Task_r_Camera,Task_t_Camera)
    #pos=[463.1625591300011, -86.95072066866062, 402.4959716796875, -176.01728820800784, 0.5390524864196777, 46.80111312866211]
    
    import rospy
    import RobotControl_func_ros1 as RobotControl_func
    rospy.init_node("RobotControl2",anonymous=True)
    robot = RobotControl_func.RobotControl_Func()

    robot.set_TMPos([463.1649544196957, -86.95035382249118, 402.50006103515625, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
    pos = robot.get_TMPos()
    point = EstimateCoord(229,198 , mtx , 510, Camera2Gripper,pos)  #自己點的點位  注意相機上下相反
    print(point)
    print(point[0][0])
    print(point[1][0])
    print(point[2][0])
    # x,y 相機座標要相反測試
    robot.set_TMPos([463.1649544196957-34.60012977,-86.95035382249118-75.7420336 , 402.50006103515625, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
    
    #robot.set_TMPos([88.61430884,-123.83022553, 59.74281404, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
    #robot.set_TMPos([275.78520487 ,200, 100, -176.0176544189453, 0.5390074849128723, 46.80110931396485])       
