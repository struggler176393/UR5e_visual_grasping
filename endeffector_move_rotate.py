import numpy as np
import math


def getPose_fromT(T):
	x = T[0, 3] 
	y = T[1, 3]
	z = T[2, 3]
	rx = math.atan2(T[2, 1], T[2, 2])
	ry = math.asin(-T[2, 0]) 
	rz = math.atan2(T[1, 0], T[0, 0])

	return x, y, z, rx, ry, rz


def getT_fromPose(pose):
	x, y, z, rx, ry, rz = pose
	Rx = np.mat([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
	Ry = np.mat([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
	Rz = np.mat([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])

	t = np.mat([[x], [y], [z]])

	R = Rz * Ry * Rx
	R_ = np.array(R)
	t_ = np.array(t)
	T_1 = np.append(R_, t_, axis = 1)
	
	zero = np.mat([0,0,0,1])
	T_2 = np.array(zero) 
	
	T = np.append(T_1, T_2, axis = 0)
	T = np.mat(T)

	return T

# 移动
def move_endEffector(axis, dist, pose):
    x, y, z, rx, ry, rz = pose
    Rx = np.mat([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.mat([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.mat([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])

    R = Rz * Ry * Rx

    unitVector = np.mat([[0], [0], [1]])
    T = R*unitVector

    pose_move = [0.0] * 6
    if axis == "x":
        pose_move[0]=dist
    if axis == "y":
        pose_move[1]=dist
    if axis == "z":
        pose_move[2]=dist
    T_move= getT_fromPose(pose_move)
	
    T_now = getT_fromPose([x, y, z, rx, ry, rz])
    T_moveTarget = T_now*T_move
    pose = getPose_fromT(T_moveTarget)
    output_x = pose[0]
    output_y = pose[1]
    output_z = pose[2]
    rx = pose[3]
    ry = pose[4]
    rz = pose[5]

    return [output_x, output_y, output_z, rx, ry, rz]

#旋转
def rotate_endEffector(axis, angle, pose):
    x, y, z, rx, ry, rz = pose
    Rx = np.mat([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.mat([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.mat([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])

    R = Rz * Ry * Rx

    unitVector = np.mat([[0], [0], [1]])
    T = R*unitVector

    pose_spin = [0.0] * 6
    if axis == "x":
        pose_spin[3]=angle
    if axis == "y":
        pose_spin[4]=angle
    if axis == "z":
        pose_spin[5]=angle
    # print(axis)
    # print(pose_spin)
    T_spin= getT_fromPose(pose_spin)
    # print(T_spin)
    T_now = getT_fromPose([x, y, z, rx, ry, rz])
    # print(T_now)
    T_spinTarget = T_now*T_spin
    # print(T_spinTarget)
    pose = getPose_fromT(T_spinTarget)
    output_x = pose[0]
    output_y = pose[1]
    output_z = pose[2]
    rx = pose[3]
    ry = pose[4]
    rz = pose[5]

    return [output_x, output_y, output_z, rx, ry, rz]



# pose_now = [-0.072944147641399, -0.06687830562048944, 0.4340418493881254, -0.2207496117519063, 0.0256862005614321, 0.1926014162476009]

# # 得到目标位置的位姿
# # 沿x轴移动5cm
# pose_move_x = move_endEffector("x", 0.05, pose_now)
# print(pose_move_x)

# # 沿y轴移动-5cm
# pose_move_y = move_endEffector("y", -0.05, pose_now)
# print(pose_move_y)

# # 沿z轴旋转0.05弧度
# pose_rotate_z = rotate_endEffector("z", 0.05, pose_now)
# print(pose_rotate_z)

# # 沿x轴旋转-0.05弧度
# pose_rotate_x = rotate_endEffector("x", -0.05, pose_now)
# print(pose_rotate_x)