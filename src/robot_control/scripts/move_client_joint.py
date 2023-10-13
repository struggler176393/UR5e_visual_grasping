#!/usr/bin/env python
import rospy
from robot_control.srv import MovePoints_joint,MovePoints_jointRequest,MovePoints_jointResponse
from geometry_msgs.msg import Pose, PoseArray
import copy

def main():
    rospy.init_node("move_points_client_python_joint")
    client = rospy.ServiceProxy("move_points", MovePoints_joint)
    rospy.wait_for_service("move_points")
    
    srv = MovePoints_jointRequest()
    pose = [-0.11959, -1.19192, 2.42700, -3.37352, -1.73175, -1.25805,
                 -0.11962, -1.191879, 1.786, -2.13026, -1.5618, -1.2581,
                 -0.11962, -1.19196, 1.1555, -1.4718, -1.5618, -1.258,
                 -0.11954, -1.19190, 0.4530, -0.937, -1.561, -1.258,
                 -0.11956, -1.1918, 0.1115, -1.155, -1.3586, -1.258,
                 -0.1194, -1.1919042629054566, 0.111, -2.02, -1.358636204396383, -1.2581217924701136,
                 -0.11955815950502569, -1.191875920896866, -0.23125745356082916, -0.6156986516765137, -0.4870050589190882, -1.2580526510821741,
                 -0.11955863634218389, -1.1918563407710572, 0.38101369539369756, -2.450367113152975, -1.1800411383258265, -1.2580350081073206,
                 -0.1195300261126917, -1.191911832695343, 0.3810990492450159, -1.297207997446396, -1.2170818487750452, -1.3169639746295374]
    




    srv.goal = pose
    srv.scale = 2
    srv.point_num = 9

    
        
    try:
        response = client(srv)
        current_pose = response.current_pose
        rospy.loginfo("Received response. Current pose: (%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call service move_points: %s", str(e))
        return 1

    return 0

if __name__ == "__main__":
    main()


