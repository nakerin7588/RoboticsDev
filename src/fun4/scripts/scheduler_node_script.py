#!/usr/bin/python3

# ROS 2 imports
import rclpy
from rclpy.node import Node

# Additional library
import math
import numpy as np

# Additional msg srv
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from fun4_interfaces.srv import SetModePosition

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        
        # Declare parameters for node configuration
        self.declare_parameter('rate', 100)
        self.declare_parameter('trajtime', 5.0)
        
        # Initialize parameters from the declared parameters
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        self.trajtime = self.get_parameter('trajtime').get_parameter_value().double_value
        
        # Initialize timer
        self.create_timer(1/self.rate, self.timer_callback)

        # ROS 2 Topic publisher setup
        self.taskspace_target_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.q_init_to_dk = self.create_publisher(Vector3, '/q_init_to_dk', 10)
        
        # ROS 2 Topic subscriber setup
        self.create_subscription(Vector3, '/dk_config_space_world', self.dk_config_space_world_callback, 10)
        self.create_subscription(Vector3, '/dk_config_space_eff', self.dk_config_space_eff_callback, 10)
        
        # ROS 2 Service server setup
        self.create_service(SetModePosition, '/mode_select', self.mode_select_callback)
        self.create_service(SetModePosition, '/ik_config_space', self.ik_config_space_callback)
        
        # ROS 2 Service clients setup
        self.random_client = self.create_client(SetModePosition, '/random_target')
        self.ik_target_client = self.create_client(SetModePosition, '/ik_target')
        
        # Initialize a variables
        self.mode = 0 # 0 = wait, 1 = inverse kinematic, 2 = teleop_twist_ref_eff_frame, 3 = teleop_twist_ref_world_frame, 4 = auto_generate
        self.ik_target = [0.0, 0.0, 0.0]
        self.dk_config_space_world = [0.0, 0.0, 0.0]
        self.dk_config_space_eff = [0.0, 0.0, 0.0]
        self.joint_name = ['joint_1', 'joint_2', 'joint_3']
        self.joint_target = [0.0, 0.0, 0.0]
        self.joint_current = [0.0, 0.0, 0.0]
        self.trajc = [0, 0, 0]
        self.trajp = [0.0, 0.0, 0.0]
        self.trajisfinish = np.array([1, 1, 1])
        
        self.get_logger().info("Scheduler node has been started")
    
    # Define function
    def mode_select_callback(self, request, response):
        try:
            if request.mode == 0:
                self.mode = 0
                response.success = True
                self.get_logger().info("Success to change mode")
                self.get_logger().info("Now is waiting mode")
                return response
            elif request.mode == 1:
                self.mode = 1
                response.success = True
                self.get_logger().info("Success to change mode")
                self.get_logger().info("Now is inverse kinematic mode")
                return response
            elif request.mode == 2:
                self.mode = 2
                response.success = True
                self.get_logger().info("Success to change mode")
                self.get_logger().info("Now is teleop reference velocity from end effector frame mode")
                return response
            elif request.mode == 3:
                self.mode = 3
                response.success = True
                self.get_logger().info("Success to change mode")
                self.get_logger().info("Now is teleop reference velocity from world frame mode")
                return response
            elif request.mode == 4:
                self.mode = 4
                response.success = True
                self.get_logger().info("Success to change mode")
                self.get_logger().info("Now is auto mode")
                return response
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Mode select function has {e}")
        
    def timer_callback(self):
        try:
            if self.mode == 0 :
                pass
            elif self.mode == 1 :
                for i in range(3):
                    self.joint_current[i], self.trajc[i] = self.trajcompute(self.joint_current[i], self.joint_target[i], self.trajp[i], 0.000001, self.trajc[i], i)
            elif self.mode == 2:
                for i in range(3):
                    self.joint_current[i] = self.joint_current[i] + (self.dk_config_space_eff[i] / self.rate)
                
            elif self.mode == 3:
                for i in range(3):
                    self.joint_current[i] = self.joint_current[i] + (self.dk_config_space_world[i] / self.rate)

            elif self.mode == 4:
                if np.any(self.trajisfinish == 0):
                    for i in range(3):
                        self.joint_current[i], self.trajc[i] = self.trajcompute(self.joint_current[i], self.joint_target[i], self.trajp[i], 0.000001, self.trajc[i], i)
                elif np.all(self.trajisfinish == 1):
                    self.get_logger().info(f'Finish task')
                    request = SetModePosition.Request()
                    future = self.random_client.call_async(request)
                    future.add_done_callback(self.get_random)   
                    self.trajisfinish[:] = 2
                    
            self.publish_q_init_to_dk()
            self.publish_joint_state(self.joint_current)
        except Exception as e:
            self.publish_joint_state(self.joint_current)
            self.get_logger().error(f"SchedulerNode has {e}")
    
    def get_random(self, future):
        try:
            response = future.result()
            request = SetModePosition.Request()
            request.position.x = response.position.x
            request.position.y = response.position.y
            request.position.z = response.position.z
            self.ik_target_client.call_async(request)
            future.add_done_callback(self.send_ik_target)  
        except Exception as e:
            self.get_logger().error(f"Get random function has {e}")      
        
    def send_ik_target(self, future):
        try:
            response = future.result()
            if response.success == False:
                self.get_logger().error(f"Can not receive target!. Reset mode")
                self.mode = 0
        except Exception as e:
            self.get_logger().error(f"Send inverse kinematic target function has {e}")
            
    def ik_config_space_callback(self, request, response):
        try:
            for i in range(3):
                self.trajc[i] = 0
            self.joint_target[0] = request.position.x
            self.joint_target[1] = request.position.y
            self.joint_target[2] = request.position.z
            self.get_logger().info(f'target joint angle are :{self.joint_target}')
            for i in range(3):
                self.trajp[i] = self.normalize_angle(math.fabs(self.joint_current[i] - self.joint_target[i]))     
            self.trajisfinish[:] = 0
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error(f"Inverse kinematic space call back has {e}")
            response.success = False
            return response   
        
    def trajcompute(self, q, target, path, tol, count, i):
        times = int(self.trajtime * self.rate)
        try:
            if math.fabs(q - target) <= tol or count == times:
                self.trajisfinish[i] = 1
                return q, count
            elif q > target:
                count += 1
                return (q-(path/times)), count
            elif q < target:
                count += 1
                return (q+(path/times)), count
        except Exception as e:
            self.get_logger().error(f"Trajectory compute function has {e}")
            return q, count

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def dk_config_space_world_callback(self, msg):
        # self.get_logger().info(f'{msg}')
        self.dk_config_space_world[0] = msg.x
        self.dk_config_space_world[1] = msg.y
        self.dk_config_space_world[2] = msg.z
    
    def dk_config_space_eff_callback(self, msg):
        # self.get_logger().info(f'{msg}')
        self.dk_config_space_eff[0] = msg.x
        self.dk_config_space_eff[1] = msg.y
        self.dk_config_space_eff[2] = msg.z
    
    def publish_q_init_to_dk(self):
        try:
            msg = Vector3()
            msg.x = self.joint_current[0]
            msg.y = self.joint_current[1]
            msg.z = self.joint_current[2]
            self.q_init_to_dk.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Publish q init to dk function has {e}")
        
    def publish_joint_state(self, q):
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = q
            msg.name = self.joint_name
            self.taskspace_target_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Publish joint state function has {e}")
    
def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
