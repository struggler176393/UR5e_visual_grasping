#include "ros/ros.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

#include <geometry_msgs/Pose.h>
#include <robot_control/MovePoints_joint.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

ros::MultiThreadedSpinner spinner(2);


void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale){
  int n_joints = plan.trajectory_.joint_trajectory.joint_names.size();
  for(int i=0;i<plan.trajectory_.joint_trajectory.points.size();i++){
    plan.trajectory_.joint_trajectory.points[i].time_from_start *= 1/scale;
    for(int j=0;j<n_joints;j++){
      plan.trajectory_.joint_trajectory.points[i].velocities[j] *= scale;
      plan.trajectory_.joint_trajectory.points[i].accelerations[j] *= scale*scale;
    }
  }
}





bool movePointsCallback(robot_control::MovePoints_joint::Request& req,
                        robot_control::MovePoints_joint::Response& res)
{
  moveit::planning_interface::MoveGroupInterface group("manipulator");//ur5对应moveit中选择的规划部分
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
  



  moveit::core::RobotStatePtr start_state(group.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(group.getName());
  std::vector<double> joint_group_positions;
  start_state->copyJointGroupPositions(joint_model_group,joint_group_positions);





  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  

  int i;
  for (i=0;i<req.point_num;i++) {

      std::vector<double> pose(6);
      pose[0] = req.goal[i*6+0];
      pose[1] = req.goal[i*6+1];
      pose[2] = req.goal[i*6+2];
      pose[3] = req.goal[i*6+3];
      pose[4] = req.goal[i*6+4];
      pose[5] = req.goal[i*6+5];
      
      int j;
      for (int j = 0; j < 6; ++j)  
      {  
        ROS_INFO("my_array[%d] = %f", j, pose[j]);  
      }  



      

      group.setJointValueTarget(pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::planning_interface::MoveItErrorCode success = group.plan(plan);
      plans.push_back(plan);


      moveit::core::RobotStatePtr middle_state(group.getCurrentState());
      joint_model_group = middle_state->getJointModelGroup(group.getName());
      middle_state->setJointGroupPositions(joint_model_group,pose);
      group.setStartState(*middle_state);
  }

  moveit_msgs::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = plans[0].trajectory_.joint_trajectory.joint_names;
  trajectory.joint_trajectory.points = plans[0].trajectory_.joint_trajectory.points;
  


  for (i=1;i<plans.size();i++)  
    {  
        moveit::planning_interface::MoveGroupInterface::Plan plan = plans[i]; 
        for (size_t j=1;j<plan.trajectory_.joint_trajectory.points.size();j++){
          trajectory.joint_trajectory.points.push_back(plan.trajectory_.joint_trajectory.points[j]);
        }

    }

  moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(rt);
  

  
  





  rt.getRobotTrajectoryMsg(trajectory);
  joinedPlan.trajectory_ = trajectory;


  scale_trajectory_speed(joinedPlan,req.scale);


  

  // 执行运动
  if (!group.execute(joinedPlan)){
    ROS_ERROR("Failed to execute plan");
    return false;
  }
  // sleep(1);



  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_points_server_joint");
  ros::NodeHandle node_handle; 
  ros::ServiceServer service = node_handle.advertiseService("move_points", movePointsCallback);
  // ros::spin();
  spinner.spin();
  
  return 0;
}
