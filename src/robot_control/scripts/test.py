import rospy
from robot_control.srv import MovePoints
from geometry_msgs.msg import Pose, PoseArray


pose = [Pose()]*6
print(pose[0])