#!/usr/bin/python3

# ROS 2 imports
import rclpy
from rclpy.node import Node

# Additional library
import math
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

# Additional msg srv
from fun4_interfaces.srv import SetModePosition

class InverseKinematicNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematic_node')
        
        # Declare parameters for node configuration
        self.declare_parameter('l1', 0.2)   # Length of first link (l1)
        self.declare_parameter('l2', 0.25)  # Length of second link (l2)
        self.declare_parameter('l3', 0.28)  # Length of third link (l3)
        
        # Initialize parameters from the declared parameters
        self.l1 = self.get_parameter('l1').get_parameter_value().double_value
        self.l2 = self.get_parameter('l2').get_parameter_value().double_value
        self.l3 = self.get_parameter('l3').get_parameter_value().double_value
        
        # ROS 2 Service server setup
        self.create_service(SetModePosition, '/ik_target', self.computeRRRIK)
        
        # ROS 2 Service clients setup
        self.ik_config_space_client = self.create_client(SetModePosition, '/ik_config_space')
        self.ik_target_client = self.create_client(SetModePosition, '/ik_target_display')
        
        # Initialize a variables
        self.ik_config_space_var = [0.0, 0.0, 0.0]
        
        # Define the robot using Modified Denavit-Hartenberg parameters
        mdh = [[0.0, 0.0, 0.2, 0], [0.0, np.pi/2.0, 0.12, np.pi/2.0], [0.25, 0.0, -0.1, -np.pi/2.0]]
        revjoint = []
        
        # Create revolute joints based on MDH parameters
        for data in mdh:
            revjoint.append(rtb.RevoluteMDH(a=data[0], alpha=data[1], d=data[2], offset=data[3])) # Append each revolute joint
        
        # Create a DHRobot object representing a 3-DOF robot arm
        self.robot = rtb.DHRobot(
            [
                revjoint[0],
                revjoint[1],
                revjoint[2]
            ],
            tool = SE3.Rx(-np.pi/2) * SE3.Tz(0.28),
            name="3R robot"
        )
        
        self.get_logger().info("Inverse kinematic node has been started")
    
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def computeRRRIK(self, request, response):
        '''
        Compute RRR inverse kinematic function.
        This function is calculate joint position from taskspace and compute it via robotics toolbox inverse kinematic.
        And also send the target joint position to scheduler node for calculate trajectory and publish current joint position to rviz.
        '''
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
            self.get_logger().error(f"Compute RRR inverse kinematic has {e}")
            response.success = False
            response.message = "Cann't calculate the IK."
            return response 

    def send_joint_state(self):
        try:
            position = SetModePosition.Request()
            position.position.x = self.ik_config_space_var[0]
            position.position.y = self.ik_config_space_var[1]
            position.position.z = self.ik_config_space_var[2]
            self.ik_config_space_client.call_async(position)
        except Exception as e:
            self.get_logger().error(f"Send joint state function has {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
