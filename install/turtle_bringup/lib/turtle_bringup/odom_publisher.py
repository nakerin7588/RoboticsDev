#!/usr/bin/python3

from turtle_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

import numpy as np
import math

class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10) #subscriber
        self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, 10) #subscriber
        
        self.odom1_publisher = self.create_publisher(Odometry, '/odom1', 10) #publisher odom
        self.odom2_publisher = self.create_publisher(Odometry, '/odom2', 10) #publisher odom
        self.tf_boardcaster = TransformBroadcaster(self) #transform
        
        self.turtle1_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        self.turtle2_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        
        
    def turtle1_pose_callback(self, msg):
        # self.turtle1_pose[0] = msg.x
        # self.turtle1_pose[1] = msg.y
        # self.turtle1_pose[2] = msg.theta
        self.example_pub(msg, "turtle1",self.odom1_publisher)
        
    def turtle2_pose_callback(self, msg):
        # self.turtle2_pose[0] = msg.x
        # self.turtle2_pose[1] = msg.y
        # self.turtle2_pose[2] = msg.theta
        self.example_pub(msg, "turtle2", self.odom2_publisher)
        
    def example_pub(self, msg, child_frame_id, publisher):
        
        # robot_pose = self.turtle_name
        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = child_frame_id
        #odom position--------------------------------------
        odom_msg.pose.pose.position.x = msg.x
        odom_msg.pose.pose.position.y = msg.y
        #odom orentation------------------------------------
        q = quaternion_from_euler(0, 0, msg.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        publisher.publish(odom_msg)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = child_frame_id
         #transform position-----------------------------------
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        #transform orentation----------------------------------
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_boardcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
