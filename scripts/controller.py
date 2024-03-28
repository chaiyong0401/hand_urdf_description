#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from hand_urdf_description.msg import JointSet
import random

def joint_set_publisher():
    pub = rospy.Publisher('/mujoco_ros_interface/joint_set', JointSet, queue_size=10)
    rospy.init_node('joint_set_publisher_node')
    rate = rospy.Rate(100)  # 1hz

    control_tick_ = 0

    while not rospy.is_shutdown():
        joint_set_msg = JointSet()
        joint_set_msg.header = Header()
        joint_set_msg.header.stamp = rospy.Time.now()
        joint_set_msg.time = rospy.get_time()
        joint_set_msg.MODE = 0
        # joint_set_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        random_values = [random.uniform(-0.002, 0.002) for _ in range(16)]
        joint_set_msg.position = random_values
        # joint_set_msg.position[0:4] = [0.0, 0.0, 0.0, 0.0]
        # joint_set_msg.position[4:8] = [0.0, 0.0, 0.0, 0.0]
        # joint_set_msg.position[8:12] = [0.0, 0.0, 0.0, 0.0]
        # joint_set_msg.position[12:16] = [1.5708 * (1-1/(control_tick_%100+1)), \
        #                                  1.5708 * (1-1/(control_tick_%100+1)), \
        #                                  1.5708 * (1-1/(control_tick_%100+1)), \
        #                                  1.5708 * (1-1/(control_tick_%100+1))]
        # joint_set_msg.position[12:16] = [0, \
        #                                  0.05, \
        #                                  0.1, \
        #                                  0.1]
        joint_set_msg.position[0] = 0.0
        joint_set_msg.position[1] = 0.0
        joint_set_msg.position[4] = 0.0
        joint_set_msg.position[5] = 0.0
        joint_set_msg.position[8] = 0.0
        joint_set_msg.position[9] = 0.0
        joint_set_msg.position[12] = 0.0
        joint_set_msg.position[13] = 0.0
        joint_set_msg.torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 로그 출력 및 메시지 publish
        # rospy.loginfo(joint_set_msg)
        pub.publish(joint_set_msg)
        rate.sleep()
        control_tick_ =+ 1

if __name__ == '__main__':
    try:
        joint_set_publisher()
    except rospy.ROSInterruptException:
        pass