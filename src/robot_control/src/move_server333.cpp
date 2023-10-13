#include "ros/ros.h"
#include "robot_control/MovePoints.h"
#include <iostream>
#include <cmath>


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 



#include <geometry_msgs/Pose.h>
#include <robot_control/MovePoints.h>

// bool movePointsCallback(robot_control::MovePoints::Request& req,
//                         robot_control::MovePoints::Response& res)
// {
//   // 创建moveit控制机械臂的接口
  
  // moveit::planning_interface::MoveGroupInterface move_group("arm");
  // move_group.setGoalJointTolerance(0.001);
  // move_group.setGoalPositionTolerance(0.001);
  // move_group.setGoalOrientationTolerance(0.01);
  // move_group.setPoseReferenceFrame("base");
  // move_group.setPlanningTime(5);
  // move_group.allowReplanning(true);
  // move_group.setPlannerId("RRTConnect");
  // move_group.setMaxAccelerationScalingFactor(1);
  // move_group.setMaxVelocityScalingFactor(1);


//   double targetPose[6] = {-0.12174417293871631+25/180*M_PI, -1.548835405419073, 1.0568126924397783, -2.693364369465602, -2.956528061980836, -1.6631575702179635};
//   //关节的向量，赋值
//   std::vector<double> joint_group_positions(6);
//   joint_group_positions[0] = targetPose[0];
//   joint_group_positions[1] = targetPose[1];
//   joint_group_positions[2] = targetPose[2];
//   joint_group_positions[3] = targetPose[3];
//   joint_group_positions[4] = targetPose[4];
//   joint_group_positions[5] = targetPose[5];
  
//   //将关节值写入
//   move_group.setJointValueTarget(joint_group_positions);
//   move_group.move(); //规划+移动
//   sleep(1);



//   // geometry_msgs::Pose pose = req.goal[0];
//   geometry_msgs::Pose target_pose1;
  


//   target_pose1.orientation.x= 1.58480958e-17;
//   target_pose1.orientation.y = 9.65925826e-01;
//   target_pose1.orientation.z = -2.58819045e-01;
//   target_pose1.orientation.w = 5.91458986e-17;
 
//   target_pose1.position.x = -0.5434919310337578;
//   target_pose1.position.y =  -0.06372072557135472;
//   target_pose1.position.z = 0.3647938827703172;
 
//   move_group.setPoseTarget(target_pose1);
//   move_group.setMaxVelocityScalingFactor(0.02);
 
 
//   // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = move_group.plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS;;
 
//   ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   
 
//   //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
//   if(success)
//       move_group.execute(my_plan);
//   else
//       ROS_ERROR("Failed to plan for pose");


//   // 设置机械臂的目标位置和姿态
//   // for (const geometry_msgs::Pose& pose : req.goal) {
    
//   //   move_group.setPoseTarget(pose);

//   //   // 进行运动规划
//   //   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   //   bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   //   if (!success) {
//   //     ROS_ERROR("Failed to plan for pose");
//   //     return false;
//   //   }

//   //   // 执行规划的路径
//   //   move_group.execute(plan);
//   //   sleep(1);

//   //   // 获取当前机械臂末端位姿
//   //   geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
//   //   res.current_pose = current_pose.pose;
//   // }

//   return true;
// }


class movePoints{
public:
  movePoints(std::string name)
  {
    // moveit::planning_interface::MoveGroupInterface group("arm");//ur5对应moveit中选择的规划部分
    group_ptr_ = new moveit::planning_interface::MoveGroupInterface(name);
    group_ptr_->setGoalJointTolerance(0.001);
    //设置允许的最大速度和加速度
    group_ptr_->setMaxAccelerationScalingFactor(1);
    group_ptr_->setMaxVelocityScalingFactor(1);
    ros::ServiceServer service = nh.advertiseService("move_points", &movePoints::movePointsCallback,this);
    ROS_INFO("666");
  }

  ~movePoints(void)
  {
  }

  bool movePointsCallback(robot_control::MovePoints::Request &req,
                     robot_control::MovePoints::Response &res){
    geometry_msgs::Pose target_pose1;
    ROS_INFO("2");
    


    target_pose1.orientation.x= 1.58480958e-17;
    target_pose1.orientation.y = 9.65925826e-01;
    target_pose1.orientation.z = -2.58819045e-01;
    target_pose1.orientation.w = 5.91458986e-17;
  
    target_pose1.position.x = -0.5434919310337578;
    target_pose1.position.y =  -0.06372072557135472;
    target_pose1.position.z = 0.3647938827703172;
    ROS_INFO("3");
  
    group_ptr_->setPoseTarget(target_pose1);
    group_ptr_->setMaxVelocityScalingFactor(1);
    ROS_INFO("4");
  
    // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("5");
  //bool success = (ptr_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    bool success = group_ptr_->plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("6");
  
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   
  
    //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
    if(success)
        group_ptr_->execute(my_plan);
    else
        ROS_ERROR("Failed to plan for pose");




        
    
    // ros::shutdown(); 
    return 0;
  }

  protected:
    moveit::planning_interface::MoveGroupInterface *group_ptr_;
    ros::NodeHandle nh;
  
};


// bool movePointsCallback(robot_control::MovePoints::Request& req,
//                         robot_control::MovePoints::Response& res)
// {
//   moveit::planning_interface::MoveGroupInterface group("arm");//ur5对应moveit中选择的规划部分
//   group.setGoalJointTolerance(0.001);
//   //设置允许的最大速度和加速度
//   group.setMaxAccelerationScalingFactor(1);
//   group.setMaxVelocityScalingFactor(1);




//   double targetPose[6] = {-0.12174417293871631+25/180*M_PI, -1.548835405419073, 1.0568126924397783, -2.693364369465602, -2.956528061980836, -1.6631575702179635};
//   //关节的向量，赋值
//   std::vector<double> joint_group_positions(6);
//   joint_group_positions[0] = targetPose[0];
//   joint_group_positions[1] = targetPose[1];
//   joint_group_positions[2] = targetPose[2];
//   joint_group_positions[3] = targetPose[3];
//   joint_group_positions[4] = targetPose[4];
//   joint_group_positions[5] = targetPose[5];
  
//   //将关节值写入
//   group.setJointValueTarget(joint_group_positions);
//   ROS_INFO("6");
//   group.move(); //规划+移动
//   ROS_INFO("6");
//   sleep(1);
//   ROS_INFO("1");






//   // 设置发送的数据，对应于moveit中的拖拽
//   geometry_msgs::Pose target_pose1;
//   ROS_INFO("2");
  


//   target_pose1.orientation.x= 1.58480958e-17;
//   target_pose1.orientation.y = 9.65925826e-01;
//   target_pose1.orientation.z = -2.58819045e-01;
//   target_pose1.orientation.w = 5.91458986e-17;
 
//   target_pose1.position.x = -0.5434919310337578;
//   target_pose1.position.y =  -0.06372072557135472;
//   target_pose1.position.z = 0.3647938827703172;
//   ROS_INFO("3");
 
//   group.setPoseTarget(target_pose1);
//   group.setMaxVelocityScalingFactor(1);
//   ROS_INFO("4");
 
//   // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   ROS_INFO("5");
// //bool success = (ptr_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
//   bool success = group.plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS;
//   ROS_INFO("6");
 
//   ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   
 
//   //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
//   if(success)
//       group.execute(my_plan);
//   else
//       ROS_ERROR("Failed to plan for pose");




      
  
//   // ros::shutdown(); 
//   return 0;
// }





int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_points_server");
  

  ROS_INFO("Ready to move robot arm to points and return current pose.");
  movePoints a("arm");

  ros::spin();
  
  

  return 0;
}
