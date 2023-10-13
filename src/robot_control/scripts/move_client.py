#!/usr/bin/env python
import rospy
from robot_control.srv import MovePoints, MovePointsRequest, MovePointsResponse  
from geometry_msgs.msg import Pose, PoseArray
import copy

def main():
    rospy.init_node("move_points_client_python")
    client = rospy.ServiceProxy("move_points", MovePoints)
    rospy.wait_for_service("move_points")
    
    srv = MovePointsRequest()
    pose = [Pose()]*6
    
    pose[0].position.x = -0.5434919310337578
    pose[0].position.y = -0.06372072557135472
    pose[0].position.z = 0.3647938827703172
    pose[0].orientation.x = 1.58480958e-17
    pose[0].orientation.y = 9.65925826e-01
    pose[0].orientation.z = -2.58819045e-01
    pose[0].orientation.w = 5.91458986e-17

    pose[1] = copy.deepcopy(pose[0])
    pose[1].position.z = pose[1].position.z + 0.1

    for i in range(4):
        pose[i+2] = copy.deepcopy(pose[i+1])
        pose[i+2].position.z = pose[i+2].position.z - 0.1

    # srv.goal = []
    # for i in range(6):
    #     srv.goal.append(pose[i])


    srv.goal = pose
    srv.scale = 3

    
        
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


