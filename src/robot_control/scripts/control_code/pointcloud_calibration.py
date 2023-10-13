# from tf_utils import publish_calibration_tf

import rospy,sys
# sys.path.append('/home/lin/anaconda3/envs/torch110/lib/python3.8/site-packages/')
sys.path.append('/home/lin/UR5e/src/robot_control/scripts/control_code/')
sys.path.append('/home/lin/UR5e/src/robot_control/scripts/')
from pynput import keyboard
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_from_matrix, translation_matrix,concatenate_matrices
from geometry_msgs.msg import Vector3, Quaternion, Transform
import geometry_msgs.msg
import tf
from pynput import keyboard
from pynput.keyboard import Key, KeyCode
import threading
from tf_utils import get_trans
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from pointcloud_reconstruction import get_pointcloud
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from tf.transformations import quaternion_from_euler,euler_from_quaternion, euler_matrix, euler_from_matrix, translation_matrix,concatenate_matrices
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import  PlanningScene, ObjectColor,CollisionObject, AttachedCollisionObject,Constraints,OrientationConstraint
from camera_capture import color_pointcloud_capture,capture_predict
import open3d as o3d
import os
from moveit_msgs.msg import RobotTrajectory
from robot_control.srv import MovePoints, MovePointsRequest


os.environ["CUDA_VISIBLE_DEVICES"] = "0"

trajectory_points = []


object_detected = False
capture_flag = False

# calibration_xyzrpy = [0.766037,	-0.295099,	1.10878-0.512,	-2.13701,	-0.142609,	0.261697]  # base_link
# calibration_xyzrpy = [-0.766037,	0.295099,	1.10878-0.512,	-2.13701,	-0.142609,	0.261697-np.pi]  # base

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



class MoveIt_Control:
    # 初始化程序
    def __init__(self):
        # Init ros config
        moveit_commander.roscpp_initialize(sys.argv)
        

        # 初始化ROS节点
        
        self.arm = moveit_commander.MoveGroupCommander('arm')
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

        

        self.prepare(a=0.2,v=0.2)

        rospy.sleep(1.5)
        # 发布场景
        self.set_scene()  # set table
        #self.arm.set_workspace([-2,-2,0,2,2,2])  #[minx miny minz maxx maxy maxz]



        while True:
            self.glue_operation()




        # 抓取服务端，负责接收抓取位姿并执行运动
        # server = rospy.Service("moveit_grasp",grasp_pose,self.grasp_callback)
        # rospy.spin()



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
                    # self.move_l(target_pos[6:], waypoints_number=len(target_xyz)-1, speed_scale = 1)
                    self.move_l_continuous(target_pos[6:], waypoints_number=len(target_xyz)-1, speed_scale = 1)

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


    def calibration(self):
        global calibration_xyzrpy
        xyz = calibration_xyzrpy[:3]
        rpy = calibration_xyzrpy[3:]
        
        trans_matrix = translation_matrix(xyz)
        rot_matrix = euler_matrix(rpy[0],rpy[1],rpy[2])
        self.camera2base_pose_matrix = concatenate_matrices(trans_matrix,rot_matrix)  # 与np.dot(trans_matrix, qua_matrix)等价

        xyzrpy = [xyz[0],xyz[1],xyz[2], rpy[0],rpy[1],rpy[2]]

        print(self.camera2base_pose_matrix)
        print(xyzrpy)

        return self.camera2base_pose_matrix, xyzrpy
    


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
    
    def camera_detect(self):

        rospy.Subscriber('/seam_points', PointCloud2,callback)

        global trajectory_points

        target_xyz_part = []
        target_xyz = []

        for pos in trajectory_points:
            if pos[0]!=0:
                trans_matrix = translation_matrix(pos)
                object_matrix = np.dot(self.camera2base_pose_matrix, trans_matrix)
                target_xyz_part.append(object_matrix[:3,3])
            else:
                target_xyz.append(np.array(target_xyz_part))
                target_xyz_part = []

        trajectory_points = []
        return target_xyz

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
        rospy.loginfo("move_l_continuous:" + str(tool_configuration))
        client = rospy.ServiceProxy("move_points", MovePoints)
        rospy.wait_for_service("move_points")
        
        srv = MovePointsRequest()
        srv.goal = waypoints
        srv.scale = speed_scale

        try:
            response = client(srv)
            current_pose = response.current_pose
            rospy.loginfo("Received response. Current pose: (%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service move_points: %s", str(e))
            return 1



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



def publish_calibration_tf(parent_frame,child_frame):
    global publish_flag
    while publish_flag:
        global calibration_xyzrpy
        while rospy.get_time() == 0.0:
            pass



        x,y,z,rx,ry,rz = calibration_xyzrpy
        [qx,qy,qz,qw] = tf.transformations.quaternion_from_euler(rx,ry,rz)

        trans = Transform(Vector3(x,y,z), Quaternion(qx,qy,qz,qw))

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_frame
        static_transformStamped.child_frame_id = child_frame

        static_transformStamped.transform = trans


        broadcaster.sendTransform(static_transformStamped)
    # rospy.spin()





def on_press(key):
    global calibration_xyzrpy
    global key_listener
    global publish_flag
    global object_detected
    global capture_flag

    dir_keys = ['q','a',  # x
                'w','s',  # y
                'e','d',  # z
                'r','f',  # rx
                't','g',  # ry
                'y','h',  # rz
                ]
    pos_name = ['x', 'y', 'z', 'rx', 'ry', 'rz']
    for i in range(6):
        if key == KeyCode.from_char(dir_keys[i*2]):
            calibration_xyzrpy[i] += 0.01
            print(pos_name[i]+'+')
            print(calibration_xyzrpy)

        if key == KeyCode.from_char(dir_keys[i*2+1]):
            calibration_xyzrpy[i] -= 0.01
            print(pos_name[i]+'-')
            print(calibration_xyzrpy)
    
    if key == Key.tab:
        object_detected = True
        print("glue!")


    if key == Key.space:
        capture_flag = True
        print("capture!")




    # if key == Key.esc:
    #     print('stop')
    #     key_listener.stop()
    #     publish_flag = False
    #     print(calibration_xyzrpy)





def publish_colored_point_cloud1111():
    pub = rospy.Publisher('/colored_point_cloud', PointCloud2, queue_size=1)
    points = np.array([[1.0, 0, 0], [0, 0, 1.0]])  # 点的位置
    colors = np.array([[1, 0, 0], [0, 0, 1]]).astype(np.float32)  # 点的颜色
    assert len(points) == len(colors), "Points and colors must have the same length"

    data = get_pointcloud()
    print(data)
    while not rospy.is_shutdown():
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_base"
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1)
        ]
        msg.fields = fields

        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * points.shape[0]



        # 将点和颜色信息填充到消息中
        point_data = []
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i]
            # rgb = (int(r) << 16) | (int(g) << 8) | int(b)
            point_data.append([x, y, z, r, g, b])
        
        msg.data = np.asarray(point_data, dtype=np.float32).tobytes()

        # 发布消息
        pub.publish(msg)
        # print("published...")
        rospy.sleep(1.0)





# 实时拍摄预测并发布

def publish_colored_point_cloud_capture():
    global capture_flag
    pub_pcd = rospy.Publisher('/colored_point_cloud_captured', PointCloud2, queue_size=1)
    
    # data = get_pointcloud()
    # data = get_pointcloud_rvbust()

    # _,_,data = color_pointcloud_capture(ifvalid = True,add_mask = True)
    _,_,data = capture_predict()
    
    # print(data.shape)

    while not rospy.is_shutdown():
        if capture_flag:
            # _,_,data = color_pointcloud_capture(ifvalid = True,add_mask = True)
            _,_,data = capture_predict()
            capture_flag = False
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_base"
        msg.height = 1
        msg.width = len(data)
        msg.is_dense = True

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1)
        ]
        msg.fields = fields

        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * data.shape[0]

        # 将点和颜色信息填充到消息中

        msg.data = np.asarray(data, dtype=np.float32).tobytes()

        # 发布消息
        pub_pcd.publish(msg)
        rospy.sleep(1.0)
            


# 对RGBD图像进行预测并发布
from predict_code.predict import predict_seam, model
import cv2
def publish_predict_picture():
    global capture_flag
    pub_pcd = rospy.Publisher('/colored_point_cloud_captured', PointCloud2, queue_size=1)

    idx = 90
    color_img = cv2.imread('src/robot_control/scripts/gluing_dataset/color/color'+ str(idx) +'.png')
    depth_img = cv2.imread('src/robot_control/scripts/gluing_dataset/depth/depth'+ str(idx) +'.tiff',-1)

    data = predict_seam(color_img, depth_img, model, vis=False, device="cuda")
    
    # print(data.shape)

    while not rospy.is_shutdown():
        
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_base"
        msg.height = 1
        msg.width = len(data)
        msg.is_dense = True

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1)
        ]
        msg.fields = fields

        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * data.shape[0]

        # 将点和颜色信息填充到消息中

        msg.data = np.asarray(data, dtype=np.float32).tobytes()

        # 发布消息
        pub_pcd.publish(msg)
        rospy.sleep(1.0)





def thread_job1():
    key_listener = keyboard.Listener(
            on_press=on_press
        )
    key_listener.start()





if __name__ =="__main__":
    rospy.init_node('pointcloud_calibration_publisher', anonymous=False)

    add_thread1 = threading.Thread(target = thread_job1)
    add_thread1.start()

    add_thread2 = threading.Thread(target = publish_calibration_tf,args=["base","camera_base"])
    add_thread2.start()

    # add_thread3 = threading.Thread(target = publish_colored_point_cloud_capture)
    # add_thread3.start()
    add_thread3 = threading.Thread(target = publish_predict_picture)
    add_thread3.start()

    

    add_thread4 = threading.Thread(target = glue_moveit)
    add_thread4.start()



    
