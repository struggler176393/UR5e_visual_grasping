#!/usr/bin/env python  

import rospy  
from sensor_msgs.msg import JointState  
  
def joint_states_callback(msg):  
    joint_names = msg.name  
    joint_positions = msg.position  
    # 处理数据  
    print(joint_names)  
    print([joint_positions[2],joint_positions[1],joint_positions[0],joint_positions[3],joint_positions[4],joint_positions[5]])  
  
rospy.init_node('joint_states_subscriber', anonymous=True)  
joint_states_subscriber = rospy.Subscriber('/joint_states', JointState, joint_states_callback)  
  
rospy.spin()  