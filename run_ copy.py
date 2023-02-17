#!/usr/bin/python
# -*- coding: utf-8 -*-

# 参考  https://blog.csdn.net/fengyu19930920/article/details/81144042

from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
from UR5e_ikSolver.inverseKinematicsUR5e import InverseKinematicsUR5, transformRobotParameter
import numpy as np
import serial
import tf
import tf.transformations
from math import pi
import cv2
from endeffector_move_rotate import rotate_endEffector



arduino = serial.Serial("/dev/ttyACM0",1000000)
servo = str(500)
arduino.write(input("请输入：").encode())



# 姿态：刚体与OX轴的夹角rx、与OY轴的夹角ry、与OZ轴的夹角rz



JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

 
def move(target_joints_pos):
    

    #从joint_state话题上获取当前的关节角度值，因为后续要移动关节时第一个值要为当前的角度值
    joint_states = rospy.wait_for_message("joint_states",JointState)
    joints_pos = joint_states.position


    #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
    goal = FollowJointTrajectoryGoal()

    #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
    goal.trajectory = JointTrajectory()
    #goal.trajectory底下一共还有两个成员，分别是joint_names和points，先给joint_names赋值
    goal.trajectory.joint_names = JOINT_NAMES
    
    
    goal.trajectory.points=[0]*2
    goal.trajectory.points[0]=JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(0.0))
    goal.trajectory.points[1]=JointTrajectoryPoint(positions=target_joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(3.0))
    # print(target_joints_pos)
    client.send_goal(goal)
    client.wait_for_result()



# def cal_joints_pos(xyz_pos,d_angle=pi/2+pi/4,desired_solution = [0, -pi/2, 0, 0, 0, 0]):
#     target_pose = xyz_pos
#     # print(target_pose)
#     joints_pos = ik.inv_kin(target_pose, desired_solution)
#     joints_pos[5] = joints_pos[5] + d_angle
#     return joints_pos
#     # print(joints_pos)


# xyz_pos:[xyz,euler]
# euler:zyx
def cal_joints_pos(xyz_pos,desired_solution = [0, -pi/2, 0, 0, 0, 0],joint_weights = [16,15,14,13,2,1]):
    x,y,z,roll,pitch,yaw = xyz_pos
    T  =  tf.transformations.euler_matrix(roll,pitch,yaw)
    T[0][3] = x
    T[1][3] = y
    T[2][3] = z
    ikur = InverseKinematicsUR5()
    ikur.setJointWeights(joint_weights)
    ikur.setJointLimits(-2*pi, 2*pi)
    joints_pos = ikur.findClosestIK(T,desired_solution)
    return joints_pos
        

def detect_board():
    board_detected = False
    while not board_detected:
       
        try:
            listener = tf.TransformListener()
            listener.waitForTransform("base_link","board",  rospy.Time(), rospy.Duration(5))   
            # 这里要记住：要使用rospy.Time()，这样会从tf缓存中一个个读取，而不能用rospy.Time.now()，这样会从当前时间读取tf，要等很久
            t = listener.getLatestCommonTime("base_link","board")
            position, quaternion = listener.lookupTransform("base_link","board", t)
            rotvec = tf.transformations.euler_from_quaternion(quaternion)
            target_xyz_pos = [-position[0],-position[1],position[2],rotvec[0],rotvec[1],rotvec[2]]
            board_detected = True
        except:
            print("board was not detected!")
    return target_xyz_pos


def grasp():
    global client
    #初始化ros节点
    rospy.init_node("pub_action_test")

    #实例化一个action的类，命名为client，与上述client对应，话题为arm_controller/follow_joint_trajectory，消息类型为FollowJointTrajectoryAction
    client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    # client = actionlib.SimpleActionClient('/pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("Waiting for server...")
    #等待server
    client.wait_for_server()
    print("Connect to server")

    #执行move函数，发布action
    origin_joints_pos = [0, -pi/2, 0, 0, 0, 0]
    
    move(origin_joints_pos)

    box_xyz_pos = [-0.4, -0.4, 0.7, -pi/2,0,3/4*pi]
    box_xyz_pos = rotate_endEffector("z",  pi/4, box_xyz_pos)

    target_xyz_pos = detect_board()
    target_xyz_pos[0] = target_xyz_pos[0]+0.23
    target_xyz_pos[1] = target_xyz_pos[1]+0.02
    target_xyz_pos[3:6] = [-pi/2,0,pi/2]
    target_xyz_pos = rotate_endEffector("z",  pi/4,target_xyz_pos)
    
    d_height = 0.1 # 抬起高度
    target_xyz_pos_up = list(target_xyz_pos)
    target_xyz_pos_up[2] = target_xyz_pos_up[2] + d_height

    print("box_xyz_pos")
    print(box_xyz_pos)
    print("target_xyz_pos")
    print(target_xyz_pos)
    print("target_xyz_pos_up")
    print(target_xyz_pos_up)
    
    # box_xyz_pos = [-0.6, -0.4, 0.7, -pi/2,0,3/4*pi]
    
    
    print("box_joints_pos")
    box_joints_pos = cal_joints_pos(box_xyz_pos,desired_solution=[-0.3, -1, 0.86, -2.8, -0.48, 0],joint_weights = [1,1,1,130,12,11])
    print(box_joints_pos)
    
    print("target_joints_pos")
    target_joints_pos = cal_joints_pos(target_xyz_pos,desired_solution=origin_joints_pos,joint_weights = [16,15,1,3,2,1])
    print(target_joints_pos)
    
    print("target_joints_pos_up")
    target_joints_pos_up = cal_joints_pos(target_xyz_pos_up,desired_solution=target_joints_pos,joint_weights = [1,1,1,130,12,11])
    print(target_joints_pos_up)
    
    
    
    
    
    
    if (target_joints_pos is not None and target_joints_pos_up is not None):
        move(target_joints_pos_up)
        # arduino.write(input("请输入：").encode())
        move(target_joints_pos)
        arduino.write(input("请输入：").encode())
        move(target_joints_pos_up)
        move(box_joints_pos)
        arduino.write(input("请输入：").encode())
        move(origin_joints_pos)
    else:
        print("目标无解")
    
    

    
    
    
    # euler = tf.transformations.euler_from_matrix([])
    
    # # box
    # test = [-0.4, 0, 0.6, -pi/2,0,pi/2]
    # test = rotate_endEffector("z", pi/4, test)
    # print(test)
    # test_joints_pos = cal_joints_pos(test,desired_solution=[-0.3, -1, 0.86, -2.8, -0.48, 0])
    
    
    # # box
    # test = [-0.4, -0.4, 0.7, -pi/2,0,3/4*pi]
    # test = rotate_endEffector("z", pi/4, test)
    # print(test)
    # test_joints_pos = cal_joints_pos(test,desired_solution=[0.5, -1, 1, -2.8, -1, 0])
    
    # print(test_joints_pos)
    
    # move(test_joints_pos)
    
    



'''
    target_xyz_pos = board_detection.poslistener_calculater()
    target_xyz_pos[3:6] = [0.0001, -pi/2, 0.0001]
    print(target_xyz_pos)
    target_joints_pos = cal_joints_pos(target_xyz_pos)

    
    print("target_joints_pos")
    print(target_joints_pos)
 '''



if __name__ == "__main__":
    grasp()
