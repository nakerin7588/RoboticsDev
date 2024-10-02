#!/usr/bin/python3

# ROS
import rclpy
from rclpy.node import Node

# Additional library
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
import numpy as np 

# Additional msg srv
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class DifferencetialKinematicNode(Node):
    def __init__(self):
        super().__init__('differencetial_kinematic_node')
        
        # Parameters setup
        self.declare_parameter('rate', 100) # Timer interupt frequency (Hz)
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        
        # Timer for publish target and end effector pose
        self.create_timer(1/self.rate, self.timer_callback)
        
        # Topic Publisher variables
        self.q_dot_world_pub = self.create_publisher(Vector3, '/dk_config_space_world', 10)
        self.q_dot_eff_pub = self.create_publisher(Vector3, '/dk_config_space_eff', 10)
        
        # Topic Subscriber variables
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Vector3, '/q_init_to_dk', self.q_init_to_dk_callback, 10)
        
        # Variables
        self.current_joint_angles = [0.0001, 0.0001, 0.0001]
        self.eff_current_position = [[0.0001, 0.0001, 0.0001], [0.0001, 0.0001, 0.0001, 0.0001]] # xyz, wxyz
        self.joint_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_velo = np.array([0.0, 0.0, 0.0])
    
        # Start up
        # Create robot model
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
        
    def timer_callback(self):
        self.joint_velocity, w = self.compute_q_dot(self.current_joint_angles, self.linear_velo)
        msg = Vector3()
        msg.x = self.joint_velocity[0]
        msg.y = self.joint_velocity[1]
        msg.z = self.joint_velocity[2]
        self.q_dot_world_pub.publish(msg)
        self.joint_velocity, w = self.compute_q_dot(self.current_joint_angles, self.linear_velo @ self.robot.fkine(self.current_joint_angles).R)
        msg.x = self.joint_velocity[0]
        msg.y = self.joint_velocity[1]
        msg.z = self.joint_velocity[2]
        self.q_dot_eff_pub.publish(msg)
    
    def cmd_callback(self, msg):
        self.get_logger().info(f"{msg.linear}")
        self.linear_velo = [msg.linear.x, msg.linear.y, msg.linear.z]
        
    def compute_q_dot(self, joint_angles, ee_velocity, threshold=0.001):
            J = self.robot.jacob0(joint_angles) # Calculate the Jacobian matrix at the current joint configuration
            J = J[:3, :3] # Reduce jacobian. Use only translation
            J_pseudo_inv = np.linalg.pinv(J) # Compute the standard Jacobian pseudo-inverse
            joint_velocities = J_pseudo_inv @ ee_velocity
            J_ = self.robot.jacob0(np.array(joint_angles)+(joint_velocities / self.rate))
            J_ = J_[:3, :3]
            w = np.linalg.det(J_)
            if (-threshold) <= w <= threshold:
                self.get_logger().warn((f"Low manipulability ({w}). Restricting motion."))
                return np.array([0.0, 0.0, 0.0]), w
            return joint_velocities, w
     
    def q_init_to_dk_callback(self, msg):
        self.current_joint_angles[0] = msg.x
        self.current_joint_angles[1] = msg.y
        self.current_joint_angles[2] = msg.z
        
def main(args=None):
    rclpy.init(args=args)
    node = DifferencetialKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
