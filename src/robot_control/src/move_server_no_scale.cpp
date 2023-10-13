#include "ros/ros.h"
#include "robot_control/MovePoints.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

#include <geometry_msgs/Pose.h>
#include <robot_control/MovePoints.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

ros::MultiThreadedSpinner spinner(2);


geometry_msgs::Pose transformPoseToBase(geometry_msgs::Pose pose_world)  
{  
    // 将pose_world的位置信息从world坐标系下转换到base坐标系下  
    Eigen::Vector3d pos_world(pose_world.position.x, pose_world.position.y, pose_world.position.z);  
    Eigen::Matrix3d rot_base;  
    rot_base << -1, 0, 0,  
                0, -1, 0,  
                0, 0, 1;  
    Eigen::Vector3d trans_base(0, 0, 0.512);  
    Eigen::Vector3d pos_base = rot_base * (pos_world - trans_base);  
  
    // 将pose_world的姿态信息从world坐标系下转换到base坐标系下  
    Eigen::Quaterniond quat_world(pose_world.orientation.w, pose_world.orientation.x, pose_world.orientation.y, pose_world.orientation.z);  
    Eigen::Quaterniond quat_base(rot_base * quat_world.toRotationMatrix());  
  
    // 将转换后的位置和姿态信息存储到pose_base中  
    geometry_msgs::Pose pose_base;  
    pose_base.position.x = pos_base.x();  
    pose_base.position.y = pos_base.y();  
    pose_base.position.z = pos_base.z();  
    pose_base.orientation.w = quat_base.w();  
    pose_base.orientation.x = quat_base.x();  
    pose_base.orientation.y = quat_base.y();  
    pose_base.orientation.z = quat_base.z();  
  
    return pose_base;  
}  



bool movePointsCallback(robot_control::MovePoints::Request& req,
                        robot_control::MovePoints::Response& res)
{
  moveit::planning_interface::MoveGroupInterface group("arm");//ur5对应moveit中选择的规划部分
  group.setGoalJointTolerance(0.001);
  group.setGoalPositionTolerance(0.001);
  group.setGoalOrientationTolerance(0.01);
  //设置允许的最大速度和加速度
  group.setMaxAccelerationScalingFactor(1);
  group.setMaxVelocityScalingFactor(1);
  group.setPoseReferenceFrame("base");
  group.setPlanningTime(5);
  group.allowReplanning(true);
  group.setPlannerId("RRTConnect");
  




  std::vector<geometry_msgs::Pose> waypoints;
  for (const geometry_msgs::Pose& pose : req.goal) {
    	waypoints.push_back(pose);
  }



// 直线运动
  moveit::planning_interface::MoveGroupInterface::Plan plan;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
  int maxtries = 100;   //最大尝试规划次数
  int attempts = 0;     //已经尝试规划次数

  while(fraction < 1.0 && attempts < maxtries)
  {
      fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, plan.trajectory_);
      attempts++;
      
      // if(attempts % 10 == 0)
      //     ROS_INFO("Still trying after %d attempts...", attempts);
  }
  
  if(fraction == 1)
  {   
      ROS_INFO("Path computed successfully. Moving the arm.");

    // 生成机械臂的运动规划数据


    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), plan.trajectory_);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool ItSuccess = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",ItSuccess?"SUCCEDED":"FAILED");
    rt.getRobotTrajectoryMsg(plan.trajectory_);


    

    // 执行运动
    group.execute(plan);
      sleep(1);
  }
  else
  {
      ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
  }

  





/*  非直线运动
  // geometry_msgs::Pose StartPose;
  // StartPose = group.getCurrentPose().pose;
  // StartPose = transformPoseToBase(StartPose);
  // req.goal.insert(req.goal.begin(), StartPose);
  // 设置机械臂的目标位置和姿态
  for (const geometry_msgs::Pose& pose : req.goal) {
    group.setStartStateToCurrentState();
    group.setPoseTarget(pose, group.getEndEffectorLink());

      
    ROS_INFO("Received response. Current pose: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("Received response. Current pose: (%f, %f, %f, %f)", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);



    // 进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = group.plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 
    
  
    //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
    if(success){
        group.execute(my_plan);
    }
    else
        ROS_ERROR("Failed to plan for pose");
    

    sleep(2);

    // 获取当前机械臂末端位姿
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
    res.current_pose = current_pose.pose;

  }
*/

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_points_server");
  ros::NodeHandle node_handle; 
  ros::ServiceServer service = node_handle.advertiseService("move_points", movePointsCallback);
  // ros::spin();
  spinner.spin();
  
  return 0;
}