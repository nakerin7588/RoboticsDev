#!/usr/bin/python3

# ROS 2 imports
import rclpy
from rclpy.node import Node

# Additional msg srv
from sensor_msgs.msg import JointState
from fun4_interfaces.srv import SetModePosition

# Additional library
import math
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

class InverseKinematicNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematic_node')
        
        # Declare parameters
        self.declare_parameter('rate', 50)
        self.declare_parameter('l1', 0.2)
        self.declare_parameter('l2', 0.25)
        self.declare_parameter('l3', 0.28)
        self.declare_parameter('trajtime', 5.0)
        
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.l1 = self.get_parameter('l1').get_parameter_value().double_value
        self.l2 = self.get_parameter('l2').get_parameter_value().double_value
        self.l3 = self.get_parameter('l3').get_parameter_value().double_value
        self.trajtime = self.get_parameter('trajtime').get_parameter_value().double_value
        
        # Timer setup
        self.timer_ = self.create_timer(1/self.rate, self.timer_callback)
        
        # Publisher setup
        # self.IK_config_space_pub = self.create_publisher(JointState, '/ik_config_space', 10)
        
        # Server setup
        self.create_service(SetModePosition, '/ik_target', self.computeRRRIK)
        
        # Client setup
        self.ik_config_space_client = self.create_client(SetModePosition, '/ik_config_space')
        self.ik_target_client = self.create_client(SetModePosition, '/ik_target_display')
        
        # Variables
        self.ik_config_space_var = [0.0, 0.0, 0.0]
        self.t = 0.0 # timer for trajectory
        
        mdh = [[0.0, 0.0, 0.2, 0], [0.0, pi/2.0, 0.12, pi/2.0], [0.25, 0.0, -0.1, -pi/2.0]]
        revjoint = [] # Create revolute joint
        for data in mdh:
            revjoint.append(rtb.RevoluteMDH(a=data[0], alpha=data[1], d=data[2], offset=data[3])) # Append revolute joint
        self.robot = rtb.DHRobot(
            [
                revjoint[0],
                revjoint[1],
                revjoint[2]
            ]
            ,tool = SE3.Rx(-pi/2) * SE3.Tz(0.28)
            ,name="3R robot"
        )
        
        # Initialize set-up
        self.get_logger().info("Inverse kinematic node has beem started")
        
    def timer_callback(self):
        """Publishes the current joint angles at the specified rate."""
        # self.publish_joint_state()
        
        
    def normalize_angle(self, angle):
        """Normalize angle to the range [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def computeRRRIK(self, request, response):
        """
        Computes inverse kinematics for the RRR manipulator based on the task-space target.
        
        :param msg: Point message containing the target's x, y, and z coordinates.
        """
        try:
            x2 = request.position.x ** 2
            z_shifted = request.position.z - self.l1
            z2 = z_shifted ** 2 
            r = math.sqrt(x2 + z2)
            if r > (self.l2 + self.l3):
                self.get_logger().info(f'r = {r}')
                raise ValueError("Target is out of reach for the manipulator.")
            
            target = SE3(request.position.x, request.position.y, request.position.z)
            self.ik_config_space_var = self.robot.ikine_LM(Tep=target, mask=[1, 1, 1, 0, 0, 0]).q
            for i in range(3):
                self.ik_config_space_var[i] = self.normalize_angle(self.ik_config_space_var[i])
            self.send_joint_state()
            request_ik_target_display = SetModePosition.Request()
            request_ik_target_display.position.x = request.position.x
            request_ik_target_display.position.y = request.position.y
            request_ik_target_display.position.z = request.position.z
            self.ik_target_client.call_async(request=request_ik_target_display)
            response.success = True
            response.message = f'The configuration space from IK are q1: {self.ik_config_space_var[0]}, q2: {self.ik_config_space_var[1]}, q3: {self.ik_config_space_var[2]}'
            return response
            
        except ValueError as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.message = "Can not calculate the IK."
            return response 

    def send_joint_state(self):
        try:
            position = SetModePosition.Request()
            position.position.x = self.ik_config_space_var[0]
            position.position.y = self.ik_config_space_var[1]
            position.position.z = self.ik_config_space_var[2]
            self.ik_config_space_client.call_async(position)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
