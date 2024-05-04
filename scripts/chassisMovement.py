#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class TF() :
    def __init__(self) :
        rospy.init_node("tf_broadcaster")

        rospy.Subscriber("odom", Odometry, self.pose_cb)

        tf = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        self.robot_odometry = Odometry()

        dt = 0.02
        rate = rospy.Rate(int(1.0 / dt))

        while not rospy.is_shutdown() :
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.robot_odometry.header.frame_id
            t.child_frame_id = "chassis"
            t.transform.translation.x = self.robot_odometry.pose.pose.position.x
            t.transform.translation.y = self.robot_odometry.pose.pose.position.y
            t.transform.rotation.x = self.robot_odometry.pose.pose.orientation.x
            t.transform.rotation.y = self.robot_odometry.pose.pose.orientation.y
            t.transform.rotation.z = self.robot_odometry.pose.pose.orientation.z
            t.transform.rotation.w = self.robot_odometry.pose.pose.orientation.w

            tf.sendTransform(t)
            rate.sleep()

    def pose_cb(self, odometry) :
        self.robot_odometry = odometry

if __name__ == "__main__" :
    TF()