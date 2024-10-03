#!/usr/bin/python3

# ROS 2 imports
import rclpy
from rclpy.node import Node

# Additional library
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np 

# Additional msg srv
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

class DifferencetialKinematicNode(Node):
    def __init__(self):
        super().__init__('differencetial_kinematic_node')
        
        # Declare parameters for node configuration
        self.declare_parameter('rate', 100)
        
        # Initialize parameters from the declared parameters
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        
        # Initialize timer
        self.create_timer(1/self.rate, self.timer_callback)
        
        # ROS 2 Topic publisher setup
        self.q_dot_world_pub = self.create_publisher(Vector3, '/dk_config_space_world', 10)
        self.q_dot_eff_pub = self.create_publisher(Vector3, '/dk_config_space_eff', 10)
        self.teleop_status_pub = self.create_publisher(String, '/teleop_status', 10)
        
        # ROS 2 Topic subscriber setup
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Vector3, '/q_init_to_dk', self.q_init_to_dk_callback, 10)
        
        # Initialize a variables
        self.current_joint_angles = [0.0001, 0.0001, 0.0001]
        self.joint_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_velo = np.array([0.0, 0.0, 0.0])
    
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
        
        self.get_logger().info("Differencetial kinematic node has been started")
    
    def timer_callback(self):
        try:
            self.joint_velocity = self.compute_q_dot(self.current_joint_angles, self.linear_velo)
            msg = Vector3()
            msg.x = self.joint_velocity[0]
            msg.y = self.joint_velocity[1]
            msg.z = self.joint_velocity[2]
            self.q_dot_world_pub.publish(msg)
            self.joint_velocity = self.compute_q_dot(self.current_joint_angles, self.linear_velo @ self.robot.fkine(self.current_joint_angles).R)
            msg.x = self.joint_velocity[0]
            msg.y = self.joint_velocity[1]
            msg.z = self.joint_velocity[2]
            self.q_dot_eff_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Timer call back function has {e}")
        
    def compute_q_dot(self, joint_angles, ee_velocity, threshold=0.001):
        '''
        Compute joint velocity function
        This function is calculate the joint velocity from this formula below:
                            q_dot = inv_jacobian(q) * v
            Let: 
                q_dot is joint velocity
                inv_jacobian is inverse of jacobian matrix from current q position (In this program use psedo_inverse form numpy)
                v is velocity that reference from world frame
        And also calculate the det of jacobian matrix to check is that pose of manipulator has singularity from this formula below:
                    w = det(J(q)); if w is near to 0, means that manipulator has singularity
        '''
        try:
            J = self.robot.jacob0(joint_angles)
            J = J[:3, :3]
            J_pseudo_inv = np.linalg.pinv(J)
            joint_velocities = J_pseudo_inv @ ee_velocity
            J_ = self.robot.jacob0(np.array(joint_angles)+(joint_velocities / self.rate))
            J_ = J_[:3, :3]
            w = np.linalg.det(J_)
            if (-threshold) <= w <= threshold:
                msg = String()
                msg.data = "Now robot is at singularity pose"
                self.teleop_status_pub.publish(msg)
                return np.array([0.0, 0.0, 0.0])
            msg = String()
            msg.data = "Now robot is at normal pose"
            self.teleop_status_pub.publish(msg)
            return joint_velocities
        except Exception as e:
            self.get_logger().error(f"Compute q dot function has {e}")
            
    def q_init_to_dk_callback(self, msg):
        try:
            self.current_joint_angles[0] = msg.x
            self.current_joint_angles[1] = msg.y
            self.current_joint_angles[2] = msg.z
        except Exception as e:
            self.get_logger().error(f"q init to differencetial kinematic callback function has: {e}")
    
    def cmd_callback(self, msg):
        try:
            self.get_logger().info(f"{msg.linear}")
            self.linear_velo = [msg.linear.x, msg.linear.y, msg.linear.z]
        except Exception as e:
            self.get_logger().error(f"cmd velocity callback has {e}")
    
def main(args=None):
    rclpy.init(args=args)
    node = DifferencetialKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
