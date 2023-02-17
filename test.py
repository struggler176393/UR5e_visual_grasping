from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
import numpy as np
import serial
import tf
import tf.transformations
from math import pi
import cv2



import serial
arduino = serial.Serial("/dev/ttyACM0",1000000)

while True:
    arduino.write(input("请输入：").encode())
    # print(arduino.readline().decode("ASCII"))



 

# import rospy
# import numpy as np
# import inverse_kinematics as ik
# import time


# import tf.transformations as tf
# # from std_msgs.msg import String
# # import geometry_msgs.msg
# from tf2_msgs.msg import TFMessage
# global camera2robot, boardcorner2camera, boardcenter2boardcorner, boardcenter2camera
# camera2robot = tf.translation_matrix([0, 0, 0])
# boardcorner2camera = tf.translation_matrix([0, 0, 0])
# boardcenter2camera = tf.translation_matrix([0, 0, 0])
# boardcenter2boardcorner = tf.translation_matrix([0, -0.018, 0.018])




# boardcorner2camera = tf.translation_matrix([1, 1, 1])



# boardcenter2camera = np.dot(boardcorner2camera,boardcenter2boardcorner)
# print(1)
# print(boardcorner2camera)
# print(2)
# print(boardcenter2boardcorner)
# print(3)
# print(boardcenter2camera)




# board_detected = False
# # while not board_detected:
    
# rospy.init_node("pub_action_test")

# listener = tf.TransformListener()
# listener.waitForTransform("base_link","tool0",  rospy.Time(), rospy.Duration(5))   
# # 这里要记住：要使用rospy.Time()，这样会从tf缓存中一个个读取，而不能用rospy.Time.now()，这样会从当前时间读取tf，要等很久
# t = listener.getLatestCommonTime("base_link","tool0")
# position, quaternion = listener.lookupTransform("base_link","tool0", t)
# print(position)
# print(quaternion)
# rotmat = tf.transformations.quaternion_matrix(quaternion)
# euler = tf.transformations.euler_from_quaternion(quaternion)
# print(rotmat)
# print(euler)
# print(np.array(euler)*180/pi)
# tf.transformations.rotation_matrix()
# target_xyz_pos = [-position[0],-position[1],position[2],rotvec[0],rotvec[1],rotvec[2]]
# board_detected = True

    # print(target_xyz_pos)

# t = np.sqrt(2)/2
# mat = [[-t,0,-t],[t,0,-t],[0,-1,0]]
# euler = tf.transformations.euler_from_matrix(mat)
# print(euler)
# print(3/4*pi)
# print([-pi/2,0,3/4*pi])


# from UR5e_ikSolver.inverseKinematicsUR5e import InverseKinematicsUR5, transformRobotParameter

# def cal_joints_pos(xyz_pos,desired_solution = [0, -pi/2, 0, 0, 0, 0]):
#     ikur = InverseKinematicsUR5()
#     ikur.setJointLimits(-pi, pi)
#     joints_pos = ikur.findClosestIK(xyz_pos,desired_solution)
#     return joints_pos



# xyz_pos = [-0.6, 0, 0.7, -pi/2,0,3/4*pi]
# x,y,z,roll,pitch,yaw = xyz_pos
# test  =  tf.transformations.euler_matrix(roll,pitch,yaw)
# test[0][3] = x
# test[1][3] = y
# test[2][3] = z
# print(test)
# test_joints_pos = cal_joints_pos(test,desired_solution=[0.1653904353159923, -1.476869661447141, 0.6925367261917055, -2.3572597183343573, -0.9507885987134398, -1.5707963267948968])