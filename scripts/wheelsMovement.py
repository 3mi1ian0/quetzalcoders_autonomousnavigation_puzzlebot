#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class RobotMovement() :
    def __init__(self) :
        rospy.init_node("joints_movement")

        rospy.Subscriber("wl", Float32, self.wl_cb)
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        self.joints_pub = rospy.Publisher("joint_states", JointState, queue_size = 1)

        self.joints = JointState()

        self.joints.header.frame_id = "chassis"
        self.joints.name.extend(["chassis_joint_wR", "chassis_joint_wL"])
        self.joints.position.extend([0.0, 0.0]) 
        self.joints.velocity.extend([0.0, 0.0]) 
        self.joints.effort.extend([0.0, 0.0])

        self.wl = 0.0
        self.wr = 0.0

        dt = 0.02
        rate = rospy.Rate(int(1.0 / dt))

        while not rospy.is_shutdown() :
            t = rospy.get_time()
            self.joints.header.stamp = rospy.Time.now() 
            self.joints.position[0] = self.wr * t
            self.joints.position[1] = self.wl * t
            self.joints_pub.publish(self.joints)             
            rate.sleep()

    def wl_cb(self, msg) :  
        self.wl = msg.data

    def wr_cb(self, msg) :  
        self.wr = msg.data

if __name__ == "__main__" :
    RobotMovement()