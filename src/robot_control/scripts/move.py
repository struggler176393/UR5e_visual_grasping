# from tf_utils import publish_calibration_tf
from __future__ import print_function
import rospy,sys
# from pynput import keyboard
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_from_matrix, translation_matrix,concatenate_matrices
from geometry_msgs.msg import Vector3, Quaternion, Transform
import geometry_msgs.msg
import tf
# from pynput import keyboard
# from pynput.keyboard import Key, KeyCode
import threading
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from tf.transformations import quaternion_from_euler,euler_from_quaternion, euler_matrix, euler_from_matrix, translation_matrix,concatenate_matrices
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import  PlanningScene, ObjectColor,CollisionObject, AttachedCollisionObject,Constraints,OrientationConstraint
import open3d as o3d
import os
from moveit_msgs.msg import RobotTrajectory
from robot_control.srv import MovePoints_joint,MovePoints_jointRequest,MovePoints_jointResponse


os.environ["CUDA_VISIBLE_DEVICES"] = "0"

trajectory_points = []


object_detected = False
capture_flag = False



calibration_xyzrpy = [-0.77723, 0.420131, 1.15564-0.512, -2.13734, -0.156256, 0.260625-np.pi]
publish_flag = True


def callback(data):
    points = np.frombuffer(data.data, dtype=np.float32)
    points = points.reshape(-1,3)
    global trajectory_points
    xyz = []
    for p in points:
        xyz.append(list(p[:3]))
    trajectory_points = xyz

















import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep



def genCommand(char, command):
    """Update the command according to the character entered by the user."""

    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0

    #If the command entered is a int, assign this value to rPRA
    try:
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass

    # if char == 'f':
    #     command.rSP += 25
    #     if command.rSP > 255:
    #         command.rSP = 255

    if char == 'f':
        command.rSP = 255

    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0


    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255

    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command


def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
    currentCommand += '  rACT = '  + str(command.rACT)
    currentCommand += ', rGTO = '  + str(command.rGTO)
    currentCommand += ', rATR = '  + str(command.rATR)
    currentCommand += ', rPR = '   + str(command.rPR )
    currentCommand += ', rSP = '   + str(command.rSP )
    currentCommand += ', rFR = '   + str(command.rFR )


    print(currentCommand)

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)

def publisher():
    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rospy.init_node('Robotiq2FGripperSimpleController')

    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

    command = outputMsg.Robotiq2FGripper_robot_output();

    while not rospy.is_shutdown():

        command = genCommand(askForCommand(command), command)

        pub.publish(command)

        rospy.sleep(0.1)












class MoveIt_Control:
    # 初始化程序
    def __init__(self):
        # Init ros config
        moveit_commander.roscpp_initialize(sys.argv)
        

        # 初始化ROS节点
        
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        # self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.arm.set_goal_joint_tolerance(0.001)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)

        self.end_effector_link = self.arm.get_end_effector_link()
        # 设置机械臂基座的参考系
        self.reference_frame = 'base'
        self.arm.set_pose_reference_frame(self.reference_frame)

        # 设置最大规划时间和是否允许重新规划
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_planner_id("RRTConnect")

        # 设置允许的最大速度和加速度（范围：0~1）
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        

        # self.prepare(a=0.2,v=0.2)

        # rospy.sleep(1.5)
        self.client = rospy.ServiceProxy("move_points", MovePoints_joint)
        rospy.wait_for_service("move_points")




        pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        command = outputMsg.Robotiq2FGripper_robot_output()
        command = genCommand('a', command)
        pub.publish(command)

        # rospy.sleep(0.1)
        # command = genCommand('o', command)
        # pub.publish(command)
        # rospy.sleep(0.1)
        


        # self.gripper.set_joint_value_target([0.8])   # From 0.0 to 0.8
        # self.gripper.set_max_acceleration_scaling_factor(1)
        # self.gripper.set_max_acceleration_scaling_factor(1)
        # self.gripper.go(wait=True)



        # self.move_j([-0.1195915381060999, -1.1919239324382325, 2.4270058313952845, -3.3735295734801234, -1.731750790272848, -1.2580574194537562],a=3,v=3)
        # self.move_j([-0.11962014833559209, -1.191879079943039, 1.7863343397723597, -2.130263944665426, -1.5618279615985315, -1.258162800465719],a=3,v=3)
        # self.move_j([-0.11962014833559209, -1.1919607681087037, 1.1555073896991175, -1.4718886253288765, -1.5618088881122034, -1.258125130330221],a=3,v=3)
        # self.move_j([-0.11954957643617803, -1.1919009250453492, 0.45309478441347295, -0.9374885124019166, -1.5618251005755823, -1.258216683064596],a=3,v=3)
        

        pose_0 = [-0.11959, -1.19192, 2.42700, -3.37352, -1.73175, -1.25805]

        self.continuous_joint_move(pose_0,1.5)


        command = genCommand('c', command)
        command = genCommand('f', command)
        pub.publish(command)
        # rospy.sleep(0.1)



        # pose = [-0.11962, -1.191879, 1.786, -2.13026, -1.5618, -1.2581,
        #             -0.11962, -1.19196, 1.1555, -1.4718, -1.5618, -1.258,
        #             -0.11954, -1.19190, 0.4530, -0.937, -1.561, -1.258,
        #             -0.11956, -1.1918, 0.1115, -1.155, -1.3586, -1.258,
        #             -0.1194, -1.1919042629054566, 0.111, -2.02, -1.358636204396383, -1.2581217924701136,
        #             -0.1185229460345667, -1.192415015106537, -0.23114801943302155, -0.3380168241313477, -1.3805964628802698, -0.22696048418153936]

        #             # -0.11955863634218389, -1.1918563407710572, 0.38101369539369756, -2.450367113152975, -1.1800411383258265, -1.2580350081073206,
        #             # -0.1195300261126917, -1.191911832695343, 0.3810990492450159, -1.297207997446396, -1.2170818487750452, -1.3169639746295374]

        pose = [-0.11962, -1.191879, 1.786, -2.13026, -1.5618, -1.2581,
                    -0.11962, -1.19196, 1.1555, -1.4718, -1.5618, -1.258,
                    -0.11954, -1.19190, 0.4530, -0.937, -1.561, -1.258,
                    -0.11956, -1.1918, 0.1115, -1.155, -1.3586, -1.258,
                    -0.1194, -1.1919042629054566, 0.111,                            -2.02,              -1.358636204396383, -1.2581217924701136,              # 左
                    -0.11955815950502569, -1.191875920896866, -0.23125745356082916, -1.2156986516765137, -0.870050589190882, -1.2580526510821741,           #right
                    -0.11955863634218389, -1.1918563407710572, 0.38101369539369756, -2.20367113152975, -1.1800411383258265, -1.2580350081073206,
                    -0.1195300261126917, -1.191911832695343, 0.3810990492450159, -1.297207997446396, -1.2170818487750452, -1.3169639746295374]
        
        self.continuous_joint_move(pose,1.6)



        
        # srv = MovePoints_jointRequest()
        # pose = [-0.11962, -1.191879, 1.786, -2.13026, -1.5618, -1.2581,
        #             -0.11962, -1.19196, 1.1555, -1.4718, -1.5618, -1.258,
        #             -0.11954, -1.19190, 0.4530, -0.937, -1.561, -1.258,
        #             -0.11956, -1.1918, 0.1115, -1.155, -1.3586, -1.258,
        #             -0.1194, -1.1919042629054566, 0.111, -2.02, -1.358636204396383, -1.2581217924701136,
        #             -0.11955815950502569, -1.191875920896866, -0.23125745356082916, -0.6156986516765137, -0.4870050589190882, -1.2580526510821741,
        #             -0.11955863634218389, -1.1918563407710572, 0.38101369539369756, -2.450367113152975, -1.1800411383258265, -1.2580350081073206,
        #             -0.1195300261126917, -1.191911832695343, 0.3810990492450159, -1.297207997446396, -1.2170818487750452, -1.3169639746295374]

        # srv.goal = pose
        # srv.scale = 1.5
        # srv.point_num = len(pose)/6

        # try:
        #     response = self.client(srv)
        #     current_pose = response.current_pose
        #     rospy.loginfo("Received response. Current pose: (%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z)
        # except rospy.ServiceException as e:
        #     rospy.logerr("Failed to call service move_points: %s", str(e))


        
        command = genCommand('o', command)
        command = genCommand('f', command)
        pub.publish(command)


        rospy.sleep(1.51)

        self.continuous_joint_move(pose_0,1.5)


        
        # self.gripper.set_joint_value_target([0])
        # self.gripper.go(wait=True)


        # 发布场景
        # self.set_scene()  # set table
        #self.arm.set_workspace([-2,-2,0,2,2,2])  #[minx miny minz maxx maxy maxz]

        # self.move_l(end_pos, waypoints_number=1, speed_scale = 1)



        # while True:
        #     self.glue_operation()




        # 抓取服务端，负责接收抓取位姿并执行运动
        # server = rospy.Service("moveit_grasp",grasp_pose,self.grasp_callback)
        # rospy.spin()

    # def gripper


    def continuous_joint_move(self,pose,scale):
        srv = MovePoints_jointRequest()
        # pose = [-0.11959, -1.19192, 2.42700, -3.37352, -1.73175, -1.25805,
        #             -0.11962, -1.191879, 1.786, -2.13026, -1.5618, -1.2581,
        #             -0.11962, -1.19196, 1.1555, -1.4718, -1.5618, -1.258,
        #             -0.11954, -1.19190, 0.4530, -0.937, -1.561, -1.258,
        #             -0.11956, -1.1918, 0.1115, -1.155, -1.3586, -1.258,
        #             -0.1194, -1.1919042629054566, 0.111, -2.02, -1.358636204396383, -1.2581217924701136,
        #             -0.11955815950502569, -1.191875920896866, -0.23125745356082916, -0.6156986516765137, -0.4870050589190882, -1.2580526510821741,
        #             -0.11955863634218389, -1.1918563407710572, 0.38101369539369756, -2.450367113152975, -1.1800411383258265, -1.2580350081073206,
        #             -0.1195300261126917, -1.191911832695343, 0.3810990492450159, -1.297207997446396, -1.2170818487750452, -1.3169639746295374]

        srv.goal = pose
        srv.scale = scale
        srv.point_num = int(len(pose)/6)

        try:
            response = self.client(srv)
            current_pose = response.current_pose
            rospy.loginfo("Received response. Current pose: (%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service move_points: %s", str(e))




    def glue_operation(self):
        global object_detected
        while object_detected:
            print("Test for robot...")
            try:
                self.calibration()
                print(1)
                all_target_xyz = self.camera_detect()
                print(2)
                target_pos = []
                for target_xyz in all_target_xyz:
                    for xyz in target_xyz:
                        xyz[2] = xyz[2]
                        target_pos.append([xyz[0],xyz[1],xyz[2], np.pi/6, np.pi, 0])
                    
                    
                    target_pos = list(np.array(target_pos,dtype=object).flatten())
                    print(target_pos)
                    print(len(target_xyz))

                    self.move_l(target_pos[:6], waypoints_number=1, speed_scale = 1)
                    print(target_pos[:6])
                    # 涂胶speed_scale: 0.05
                    self.move_l(target_pos[6:], waypoints_number=len(target_xyz)-1, speed_scale = 1)

                    end_pos = target_pos[-6:]
                    end_pos[2] = end_pos[2]+0.02
                    self.move_l(end_pos, waypoints_number=1, speed_scale = 1)
                    rospy.sleep(2)
                    self.prepare(a=0.5,v=0.5)
                    rospy.sleep(1.5)
                    target_pos = []

                object_detected = False
                
            except:
                object_detected = False
                
                continue


    


    def close(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def scale_trajectory_speed(self, plan,scale):
        n_points = len(plan.joint_trajectory.points)
        for i in range(n_points):
            plan.joint_trajectory.points[i].time_from_start *= 1/scale
            plan.joint_trajectory.points[i].velocities = list(np.array(plan.joint_trajectory.points[i].velocities)*scale)
            plan.joint_trajectory.points[i].accelerations = list(np.array(plan.joint_trajectory.points[i].accelerations)*scale*scale)
        return plan
    

    def add_object(self, obj_id, parent_frame_id, size, position, orientation, color):
        object_id = obj_id
        self.scene.remove_world_object(object_id)
        rospy.sleep(1)
        object_size = size
        object_pose = PoseStamped()
        object_pose.header.frame_id = parent_frame_id
        object_pose.pose.position.x = position[0]
        object_pose.pose.position.y = position[1]
        object_pose.pose.position.z = position[2]
        object_pose.pose.orientation.x = orientation[0]
        object_pose.pose.orientation.y = orientation[1]
        object_pose.pose.orientation.z = orientation[2]
        object_pose.pose.orientation.w = orientation[3]
        self.scene.add_box(object_id, object_pose, object_size)
        self.setColor(object_id, color[0], color[1], color[2], color[3])
        self.sendColors()

    # 在机械臂下方添加一个table，使得机械臂只能够在上半空间进行规划和运动
    # 避免碰撞到下方的桌子等其他物体
    def set_scene(self):
        ## set table
        self.scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(1)

        orientation = [0,0,0.21643961,0.97629601]


        size = [2, 2, 0.01]
        position = [0,0,-size[2]/2]
        color = [0.5, 0.5, 0.5, 1.0]
        self.add_object('ground','world',size,position,orientation,color)

        size = [0.5, 0.5, 0.512]
        position = [0,0,size[2]/2]
        color = [1.0, 0.5, 0.5, 1.0]
        self.add_object('base_table','world',size,position,orientation,color)

        size = [0.8, 1.8, 0.760+0.044]  # 桌板高：0.044 亚克力：0.003
        position = [0.3+size[0]/2, 0, size[2]/2]
        color = [0.5, 0.5, 1.0, 1.0]
        self.add_object('desk','world',size,position,orientation,color)

        size = [0.01, 2, 2]
        position = [-0.3, 0, size[2]/2]
        color = [0.5, 1, 0.5, 1.0]
        self.add_object('wall','world',size,position,orientation,color)
        
        global calibration_xyzrpy

        size = [0.2, 0.2, 1.5]
        position = [-calibration_xyzrpy[0], -calibration_xyzrpy[1], size[2]/2]
        color = [1, 1, 1, 0.5]
        self.add_object('camera','world',size,position,orientation,color)









    # 关节规划，输入6个关节角度（单位：弧度）
    def move_j(self, joint_configuration=None,a=1,v=1):
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        if joint_configuration==None:
            joint_configuration = [0, -1.5707, 0, -1.5707, 0, 0]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        self.arm.set_joint_value_target(joint_configuration)
        rospy.loginfo("move_j:"+str(joint_configuration))
        self.arm.go()
        rospy.sleep(1)

    # 空间规划，输入xyzRPY
    def move_p(self, tool_configuration=None,a=1,v=1):
        if tool_configuration==None:
            tool_configuration = [0.3,0,0.3,0,-np.pi/2,0]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = tool_configuration[0]
        target_pose.pose.position.y = tool_configuration[1]
        target_pose.pose.position.z = tool_configuration[2]
        q = quaternion_from_euler(tool_configuration[3],tool_configuration[4],tool_configuration[5])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        rospy.loginfo("move_p:" + str(tool_configuration))
        traj = self.arm.plan()
        self.arm.execute(traj)
        rospy.sleep(1)
    

    def move_l_continuous(self, tool_configuration, waypoints_number=1, speed_scale=1):

        # 设置路点
        waypoints = []
        for i in range(waypoints_number):
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = tool_configuration[6*i+0]
            target_pose.pose.position.y = tool_configuration[6*i+1]
            target_pose.pose.position.z = tool_configuration[6*i+2]
            q = quaternion_from_euler(tool_configuration[6*i+3],tool_configuration[6*i+4],tool_configuration[6*i+5])
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            waypoints.append(target_pose.pose)
        rospy.loginfo("move_l:" + str(tool_configuration))
        self.arm.set_start_state_to_current_state()
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,  # waypoint poses，路点列表
                0.001,  # eef_step，终端步进值
                0.00,  # jump_threshold，跳跃阈值
                True)  # avoid_collisions，避障规划
            attempts += 1
            # print(fraction)
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            
            plan = self.scale_trajectory_speed(plan,speed_scale)

            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo(
                "Path planning failed with only " + str(fraction) +
                " success after " + str(maxtries) + " attempts.")
        rospy.sleep(1)




    # 空间直线运动，输入(x,y,z,R,P,Y,x2,y2,z2,R2,...)
    # 默认仅执行一个点位，可以选择传入多个点位
    def move_l(self, tool_configuration, waypoints_number=1, speed_scale=1):
        if tool_configuration==None:
            tool_configuration = [0.3,0,0.3,0,-np.pi/2,0]

        # 设置路点
        waypoints = []
        for i in range(waypoints_number):
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = tool_configuration[6*i+0]
            target_pose.pose.position.y = tool_configuration[6*i+1]
            target_pose.pose.position.z = tool_configuration[6*i+2]
            q = quaternion_from_euler(tool_configuration[6*i+3],tool_configuration[6*i+4],tool_configuration[6*i+5])
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            waypoints.append(target_pose.pose)
        rospy.loginfo("move_l:" + str(tool_configuration))
        self.arm.set_start_state_to_current_state()
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,  # waypoint poses，路点列表
                0.001,  # eef_step，终端步进值
                0.00,  # jump_threshold，跳跃阈值
                True)  # avoid_collisions，避障规划
            attempts += 1
            # print(fraction)
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            
            plan = self.scale_trajectory_speed(plan,speed_scale)

            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo(
                "Path planning failed with only " + str(fraction) +
                " success after " + str(maxtries) + " attempts.")
        rospy.sleep(1)

    def go_home(self,a=1,v=1):
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        # “up”为自定义姿态，你可以使用“home”或者其他姿态
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
    
    def prepare(self,a=1,v=1):
        self.move_j([-0.12174417293871631+25/180*np.pi, -1.548835405419073, 1.0568126924397783, -2.693364369465602, -2.956528061980836, -1.6631575702179635],a=a,v=v)

    def setColor(self, name, r, g, b, a=0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()
        # 需要设置规划场景是否有差异
        p.is_diff = True
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)
  

def glue_moveit():
    moveit_server = MoveIt_Control()






# def on_press(key):
#     global calibration_xyzrpy
#     global key_listener
#     global publish_flag
#     global object_detected
#     global capture_flag

#     dir_keys = ['q','a',  # x
#                 'w','s',  # y
#                 'e','d',  # z
#                 'r','f',  # rx
#                 't','g',  # ry
#                 'y','h',  # rz
#                 ]
#     pos_name = ['x', 'y', 'z', 'rx', 'ry', 'rz']
#     for i in range(6):
#         if key == KeyCode.from_char(dir_keys[i*2]):
#             calibration_xyzrpy[i] += 0.01
#             print(pos_name[i]+'+')
#             print(calibration_xyzrpy)

#         if key == KeyCode.from_char(dir_keys[i*2+1]):
#             calibration_xyzrpy[i] -= 0.01
#             print(pos_name[i]+'-')
#             print(calibration_xyzrpy)
    
#     if key == Key.tab:
#         object_detected = True
#         print("glue!")


#     if key == Key.space:
#         capture_flag = True
#         print("capture!")







# def thread_job1():
#     key_listener = keyboard.Listener(
#             on_press=on_press
#         )
#     key_listener.start()





if __name__ =="__main__":
    rospy.init_node('move', anonymous=False)

    # add_thread1 = threading.Thread(target = thread_job1)
    # add_thread1.start()

    

    add_thread4 = threading.Thread(target = glue_moveit)
    add_thread4.start()



    
