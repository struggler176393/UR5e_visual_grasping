#include "ros/ros.h"
#include <robot_control/MovePoints.h>
#include <cstdlib>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <vector>

// argc是参数个数，命令也算一个参数
// argv是参数的值
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_points_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robot_control::MovePoints>("move_points");
  robot_control::MovePoints srv;

  std::vector<geometry_msgs::Pose> pose_array;
  geometry_msgs::Pose pose;
  pose.position.x = -0.5434919310337578;
  pose.position.y = -0.06372072557135472;
  pose.position.z = 0.3647938827703172;
  pose.orientation.x = 1.58480958e-17;
  pose.orientation.y = 9.65925826e-01;
  pose.orientation.z = -2.58819045e-01;
  pose.orientation.w = 5.91458986e-17;
  pose_array.push_back(pose);

  pose.position.z = pose.position.z+0.1;
  pose_array.push_back(pose);
  pose.position.z = pose.position.z-0.1;
  pose_array.push_back(pose);
  pose.position.z = pose.position.z-0.1;
  pose_array.push_back(pose);
  pose.position.z = pose.position.z-0.1;
  pose_array.push_back(pose);
  pose.position.z = pose.position.z-0.1;
  pose_array.push_back(pose);



  srv.request.goal=pose_array;
  if (client.call(srv)) {
    // 输出响应
    geometry_msgs::Pose currentPose = srv.response.current_pose;
    ROS_INFO("Received response. Current pose: (%f, %f, %f)", currentPose.position.x, currentPose.position.y, currentPose.position.z);
  } else {
    ROS_ERROR("Failed to call service move_points");
    return 1;
  }

  return 0;
}