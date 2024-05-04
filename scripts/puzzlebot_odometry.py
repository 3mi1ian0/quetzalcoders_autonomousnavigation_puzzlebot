#!/usr/bin/env python3 

import rospy  
import numpy as np 
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist  
from tf.transformations import quaternion_from_euler 

class PuzzlebotOdometry() :  
    def __init__(self) :  
        # INITIATING THE NODE
        rospy.init_node('puzzlebot_odometry') 

        # SUBSCRIBERS
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb) 

        # PUBLISHERS
        self.odometry_pub = rospy.Publisher('odom', Odometry, queue_size = 1) 
        self.wr_pub = rospy.Publisher('wr', Float32, queue_size = 1) 
        self.wl_pub = rospy.Publisher('wl', Float32, queue_size = 1) 

        # ROBOT CONSTANTS
        self.r = 0.05 # Puzzlebot wheel radius [m] 
        self.L = 0.19 # Puzzlebot wheel separation [m] 
        self.dt = 0.02 # Desired time to update the robot's pose [s] 

        # VARIABLES 
        self.w = 0.0  
        self.v = 0.0  
        self.x = 0.0  
        self.y = 0.0
        self.wr = 0.0
        self.wl = 0.0
        self.theta = 0.0  
        self.odometry = Odometry() 

        rate = rospy.Rate(int(1.0 / self.dt))

        while not rospy.is_shutdown() : 
            self.update_robot_pose(self.v, self.w) 
            self.odometry = self.get_odometry(self.x, self.y, self.theta, self.v, self.w) 
            [self.wl, self.wr] = self.get_wheel_speeds(self.v, self.w) 

            self.odometry_pub.publish(self.odometry) 
            self.wr_pub.publish(self.wr) 
            self.wl_pub.publish(self.wl) 
            rate.sleep() 
     
    def cmd_vel_cb(self, msg) : 
        self.v = msg.linear.x 
        self.w = msg.angular.z 

    def get_wheel_speeds(self, v, w) : 
        wl = ((2 * v) - (w * self.L)) / self.r # Left wheel angular speed in [rad/s] 
        wr =  ((2 * v) + (w * self.L)) / self.r # Right wheel angular speed in [rad/s] 
        return [wl, wr] 

    def get_odometry(self, x, y, yaw, v, w) : 
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"  
        odometry.header.stamp = rospy.Time.now() 
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, yaw)
        odometry.pose.pose.orientation.x = quat[0]
        odometry.pose.pose.orientation.y = quat[1]
        odometry.pose.pose.orientation.z = quat[2]
        odometry.pose.pose.orientation.w = quat[3]
        odometry.twist.twist.linear.x = v
        odometry.twist.twist.angular.z = w   
        return odometry 

    def update_robot_pose(self, v, w) :
        self.theta = self.theta + (w * self.dt)
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        self.x = self.x + (v * np.cos(self.theta) * self.dt) 
        self.y = self.y + (v * np.sin(self.theta) * self.dt)

if __name__ == "__main__" :  
    PuzzlebotOdometry()