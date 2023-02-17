#!/usr/bin/env python

import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion, Transform
import tf.transformations
from math import pi



rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

x = 0.96
y = 0.215
z = 0.52
# 相对baselink反的
roll = -0.07+0.02
pitch = 0.6-0.53+0.09-0.06-0.02-0.03-0.02
yaw = -2.61-0.3+0.05-0.2

[qx,qy,qz,qw] = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

trans = Transform(Vector3(x,y,z), Quaternion(qx,qy,qz,qw))


# while True:
broadcaster = tf2_ros.StaticTransformBroadcaster()
static_transformStamped = geometry_msgs.msg.TransformStamped()

static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = "/base_link"
static_transformStamped.child_frame_id = "/camera_base"
#static_transformStamped.child_frame_id = "test_camera"

static_transformStamped.transform = trans

print(static_transformStamped)

broadcaster.sendTransform(static_transformStamped)


# broadcaster = tf2_ros.TransformBroadcaster()
# broadcaster.sendTransform(static_transformStamped)


# -----------------------------------#
# print(calib.transformation.transform)
# print(calib.transformation.child_frame_id)
# broadcaster = tf2_ros.TransformBroadcaster()
# broadcaster.sendTransform(static_transformStamped)
# -----------------------------------#






rospy.spin()