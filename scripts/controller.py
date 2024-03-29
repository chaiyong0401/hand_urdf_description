#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from hand_urdf_description.msg import JointSet
import random
from std_msgs.msg import Float32

class HandController:
    def __init__(self, hz):
        self.hz_ = hz
        self.control_init_ = False
        self.control_time_ = 0.0
        self.simtime_sub = rospy.Subscriber('/mujoco_ros_interface/sim_time', Float32, self.simtimecallback)
        self.pub = rospy.Publisher('/mujoco_ros_interface/joint_set', JointSet, queue_size=10)

    def simtimecallback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
        self.control_time_ = data.data
        if self.control_init_ == False:
            # rospy.loginfo("control_activated!")
            self.control_init_ = True

    def joint_set_publisher(self):
        # rospy.init_node('joint_set_publisher_node')
        # rate = rospy.Rate(hz_) 

        # control_time_ = 0.0
        # control_tick_ = 0
        # self.control_init_ = False
        # while not rospy.is_shutdown():
        if(self.control_init_):
            joint_set_msg = JointSet()
            joint_set_msg.header = Header()
            joint_set_msg.header.stamp = rospy.Time.now()
            joint_set_msg.time = self.control_time_
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
            self.pub.publish(joint_set_msg)
            # rate.sleep()
            # rospy.loginfo("Controller control_time_: %f", self.control_time_)
            # self.control_time_ = self.control_time_ + 1.0/self.hz_
            # rospy.loginfo(self.control_time_)

        self.control_init_ = False



if __name__ == '__main__':
    try:
        # controller = HandController()
        # controller.joint_set_publisher()

        rospy.init_node('joint_set_publisher_node')
        hz = 300
        controller = HandController(hz)
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            controller.joint_set_publisher()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass