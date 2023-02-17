from inverseKinematicsUR5e import InverseKinematicsUR5, transformRobotParameter
import numpy as np
from math import pi
import tf.transformations
import cv2


theta0 = [0.6,0.2,0.5,1.2,0.1,-0.1]
theta = [0.2,0.3,0.2,1.0,0.5,0.1]
gd = transformRobotParameter(theta)
print(gd)
# joint_weights = [1,1,1,1,1,1]
ikur = InverseKinematicsUR5()
# ikur.setJointWeights(joint_weights)
ikur.setJointLimits(-pi, pi)
# print ik.solveIK(gd)

r,p,y = tf.transformations.euler_from_matrix(gd[0:3][0:3])
rot = tf.transformations.euler_matrix(r,p,y)
print(rot)


print (ikur.findClosestIK(gd,theta0))