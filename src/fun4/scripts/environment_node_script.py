#!/usr/bin/python3

"""

"""

# ROS
import rclpy
from rclpy.node import Node

# Additional library
import roboticstoolbox as rtb
import random
import os
from ament_index_python.packages import get_package_share_directory
from spatialmath import SE3
from tf_transformations import euler_from_quaternion
from transforms3d import euler

# Additional msg srv
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from fun4_interfaces.srv import RequestRandomTarget
from sensor_msgs.msg import JointState

class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')
        
        # Parameters setup
        self.declare_parameter('rate', 1) # Timer interupt frequency (Hz)
        rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        
        # Timer for publish target and end effector pose
        timer_ = self.create_timer(1/rate, self.timer_callback)
        
        # Topic Publisher variables
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10) # This publiseher is for publish random task space
        self.end_effector_pub = self.create_publisher(PoseStamped, '/end_effector', 10) # This publiseher is for publish end effector pose
        
        # Topic Subscriber variables
        tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        tf_static_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, 10)
        joints_state = self.create_subscription(JointState, '/joints_state', self.joints_state_callback, 10)
        
        # Service Server variables
        random_target = self.create_service(RequestRandomTarget, '/random_target', self.random_target_callback)
        
        # Variables
        self.random_target_values = [0.0, 0.0, 0.0]
        self.joints_angle = []
        # self.eff_pose = self.forward_kinematic()
        self.t_0_e = SE3(0.0, 0.0, 0.0) * SE3.Eul(0.0, 0.0, 0.0)
        self.t_3_e = SE3(0.0, 0.0, 0.0) * SE3.Eul(0.0, 0.0, 0.0)
        
        # Start up node
        self.get_logger().info("Environment node has been started")
        self.workspace_calculate_func() # Initialize workspace calculation
        # Create robot model
        package_name = 'fun4'
        xacro_path = 'robot/visual/my-robot.xacro'
        robot_path = self.get_file_path(package_name, xacro_path)
        # print(robot_path)
        self.robot = rtb.ERobot.URDF(robot_path) # Initialize robot for kinematic model
        # Show robot
        print(self.robot)
        self.robot.plot(q=[0.011, 0.011, 0.011], backend='pyplot')
        
    # Function
    def target_publish_func(self, position):
        msg = PoseStamped()
        msg.header.frame_id = "end_effector"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        self.target_pub.publish(msg)
        
    def eff_publish_func(self, position):
        msg = PoseStamped()
        msg.header.frame_id = "end_effector"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        # print(msg)
        self.end_effector_pub.publish(msg)
    
    def timer_callback(self):
        # Publish target position
        self.target_publish_func(self.random_target_values)
        self.eff_publish_func(self.endeffector_pose_compute().t)
        # print(self.joints_angle)
        # print(self.robot.fkine(q=[0.011, 0.011, 0.011]))
    
    def tf_callback(self, msg):
        temp = []
        self.t_0_e = SE3(0.0, 0.0, 0.0) * SE3.Eul(0.0, 0.0, 0.0)
        for transform in msg.transforms:
            temp_tf = SE3(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z) * SE3.Eul(euler.quat2euler([transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]))
            temp.append(temp_tf)
        for i in temp:
            self.t_0_e = self.t_0_e * i
        
    def tf_static_callback(self, msg):
        self.t_3_e = msg.transforms.transform
    
    def endeffector_pose_compute(self):
        p_e_org = SE3(0.0, 0.0, 0.0)
        self.t_0_e = self.t_0_e * self.t_3_e
        return self.t_0_e * p_e_org
    
    def joints_state_callback(self, msg):
        self.joints_angle = msg
            
    def random_func(self):
        return random.uniform(-10.0, 10.0)
    
    def random_target_callback(self, request, response):
        # Random postion
        for i in range(3):   
            self.random_target_values[i] = self.random_func()
        # Response random position
        response.position.x = self.random_target_values[0]
        response.position.y = self.random_target_values[1]
        response.position.z = self.random_target_values[2]
        response.success = True
        # Log the response
        self.get_logger().info(f"Success to random position. The random position is x:{self.random_target_values[0]}, y:{self.random_target_values[1]}, z:{self.random_target_values[2]}")
        return response
    
    def workspace_calculate_func(self):
        min_reach = -10.0
        max_reach = 10.0
    
    def get_file_path(self, package_name, relative_path):
        try:
            # Get the path to the package
            package_path = get_package_share_directory(package_name)

            # Append the relative file path to the package path
            file_path = os.path.join(package_path, relative_path)

            if os.path.exists(file_path):
                return file_path
            else:
                raise FileNotFoundError(f"File '{relative_path}' not found in package '{package_name}'")
        except Exception as e:
            print(f"Error: {str(e)}")
    
def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
