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

class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Set parameters
        self.declare_parameter('namespace1', 'turtle1')
        # self.declare_parameter('namespace2', 'turtle2')
        self.declare_parameter('odom1', 'odom1')
        self.namespace1 = self.get_parameter('namespace1').get_parameter_value().string_value
        self.odom1 = self.get_parameter('odom1').get_parameter_value().string_value
        
        self.create_subscription(Pose, '/'+self.namespace1+'/pose', self.turtle1_pose_callback, 10) #subscriber
        # self.create_subscription(Pose, '/'+self.get_parameter('namespace2').get_parameter_value().string_value+'/pose', self.turtle2_pose_callback, 10) #subscriber
        
        self.odom1_publisher = self.create_publisher(Odometry, '/'+self.odom1, 10) #publisher odom
        # self.odom2_publisher = self.create_publisher(Odometry, '/odom2', 10) #publisher odom
        self.tf_boardcaster = TransformBroadcaster(self) #transform
        
        self.turtle1_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        # self.turtle2_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        
        self.get_logger().info(self.namespace1+' has been started')
        self.get_logger().info(self.odom1+' has been started')
        
        
    def turtle1_pose_callback(self, msg):
        self.example_pub(msg, self.namespace1, self.odom1_publisher)
        
    # def turtle2_pose_callback(self, msg):
    #     self.example_pub(msg, self.get_parameter('namespace2').get_parameter_value().string_value, self.odom2_publisher)
        
    def example_pub(self, msg, child_frame_id, publisher):
        
        # robot_pose = self.turtle_name
        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = child_frame_id
        #odom position--------------------------------------
        odom_msg.pose.pose.position.x = msg.x - 5.0
        odom_msg.pose.pose.position.y = msg.y - 5.0
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
        t.transform.translation.x = msg.x - 5.0
        t.transform.translation.y = msg.y - 5.0
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
