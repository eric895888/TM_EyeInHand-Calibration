import rospy
import RobotControl_func_ros1 as RobotControl_func
import numpy as np
import sys
rospy.init_node("RobotControl",anonymous=True)

robot = RobotControl_func.RobotControl_Func()
totallist=[]
print(robot.get_TMPos())


# print(robot.get_TMJoint())
# #robot.set_TMPos([0,300, 400, 3.069622855659274, 0, 0.7746395918047735])
# #obot.set_TMPos([-326.53572472, 215.70498953, 100, 3.069622855659274, 0, 0.7746395918047735])

# #robot.set_TMPos([100.46757778617632-125.93836885, 546.3748799610293-117.80607666, 405.3811340332031, 3.069622855659274, 0, 0.7746395918047735])

# # robot.set_TMPos([-540.462031822828, 82.8574295308757, 433.2264404296875, 3.1159658533682877, -0.021376819563037754, 0.7629543067317782])       

# from gripper import Gripper
# G = Gripper()
# G.gripper_off()
# G.gripper_on()


# # #robot.set_TMPos([500, 0, 200, 3.023982666161932, 0.028239334203143396, 0.7627013730072479])

# #robot.set_TMPos([0, 500, 400, 3.143982666161932, 0, 0])
# totallist.append(robot.get_TMPos())
# file_name = 'EIH_PosSet_val.txt'
# with open(file_name, 'a') as file:
#     for sublist in totallist: 
#         file.write(str(sublist))
#         file.write('\n')

# c2g=np.load(sys.path[0]+'/Camera2Gripper.npy')


# print(np.load('TC2G.npy')) #偏移矩陣

# print(np.load('Camera2Gripper.npy')) #旋轉矩陣

# print(np.load('INS.npy')) #內參矩陣

# print(np.matrix([1,2,3]))
