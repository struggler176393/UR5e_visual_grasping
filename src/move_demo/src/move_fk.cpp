#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

int main(int argc, char **argv)  //主函数
{
    //ros初始化节点，节点名为moveit_fk_demo
    ros::init(argc, argv, "move_fk");
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();
 
    //初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("arm");
    //设置机械臂运动的允许误差
    arm.setGoalJointTolerance(0.001);
    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);
 
    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move(); //规划+运动
    sleep(1);
 
    //定义一个数组，存放6个关节的信息
    double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
    //关节的向量，赋值
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];
    
    //将关节值写入
    arm.setJointValueTarget(joint_group_positions);
    arm.move(); //规划+移动
    sleep(1);
 
    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
    
    //关闭并退出
    ros::shutdown(); 
 
    return 0;
}
