//move_demo.cpp
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movo_moveit");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  moveit::planning_interface::MoveGroupInterface group("arm");//ur5对应moveit中选择的规划部分
  group.setGoalJointTolerance(0.001);
  //设置允许的最大速度和加速度
  group.setMaxAccelerationScalingFactor(1);
  group.setMaxVelocityScalingFactor(1);
  group.setPoseReferenceFrame("base");




  double targetPose[6] = {-0.12174417293871631+25/180*M_PI, -1.548835405419073, 1.0568126924397783, -2.693364369465602, -2.956528061980836, -1.6631575702179635};
  //关节的向量，赋值
  std::vector<double> joint_group_positions(6);
  joint_group_positions[0] = targetPose[0];
  joint_group_positions[1] = targetPose[1];
  joint_group_positions[2] = targetPose[2];
  joint_group_positions[3] = targetPose[3];
  joint_group_positions[4] = targetPose[4];
  joint_group_positions[5] = targetPose[5];
  
  //将关节值写入
  group.setJointValueTarget(joint_group_positions);
  group.move(); //规划+移动
  sleep(1);






  // 设置发送的数据，对应于moveit中的拖拽
  geometry_msgs::Pose target_pose1;
  


  target_pose1.orientation.x= 1.58480958e-17;
  target_pose1.orientation.y = 9.65925826e-01;
  target_pose1.orientation.z = -2.58819045e-01;
  target_pose1.orientation.w = 5.91458986e-17;
 
  target_pose1.position.x = -0.5434919310337578;
  target_pose1.position.y =  -0.06372072557135472;
  target_pose1.position.z = 0.3647938827703172;
 
  group.setPoseTarget(target_pose1);
  group.setMaxVelocityScalingFactor(1);
  group.setStartStateToCurrentState();
  group.setPoseTarget(target_pose1, group.getEndEffectorLink());
 
 
  // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//bool success = (ptr_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  bool success = group.plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS;
 
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   
 
  //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
  if(success)
      group.execute(my_plan);
  else
      ROS_ERROR("Failed to plan for pose");




      
  
  ros::shutdown(); 
  return 0;
}

