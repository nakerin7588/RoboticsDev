#!/usr/bin/python3

# ROS
import rclpy
from rclpy.node import Node

# Additional library
import math
import numpy as np

# Additional msg srv
from sensor_msgs.msg import JointState
from fun4_interfaces.srv import SetModePosition

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        
        # Parameters setup
        self.declare_parameter('rate', 100) # Timer interupt frequency (Hz)
        self.declare_parameter('trajtime', 5.0)
        
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value # Timer interupt frequency (Hz)
        self.trajtime = self.get_parameter('trajtime').get_parameter_value().double_value
        
        # Timer for publish target and end effector pose
        self.create_timer(1/self.rate, self.timer_callback)

        # Topic Publisher variables
        self.taskspace_target_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Topic Subscriber variables
        
        # Service Server variables
        self.create_service(SetModePosition, '/mode_select', self.mode_select_callback)
        self.create_service(SetModePosition, '/ik_config_space', self.ik_config_space_callback)
        
        # Service Client variables
        # self.random_client = self.create_client(SetModePosition, '/random_target')
        # self.ik_target_client = self.create_client(SetModePosition, '/ik_target')
        
        # Variables
        self.mode = 0 # 0 = wait, 1 = inverse kinematic, 2 = teleop_twist_ref_eff_frame, 3 = teleop_twist_ref_world_frame, 4 = auto_generate
        self.ik_target = [0.0, 0.0, 0.0]
        self.joint_name = ['joint_1', 'joint_2', 'joint_3']
        self.joint_target = [0.0, 0.0, 0.0]
        self.joint_current = [0.0, 0.0, 0.0]
        self.trajc = [0, 0, 0]
        self.trajp = [0.0, 0.0, 0.0]
        self.trajisfinish = 1
        
        # Start up
        

    
    def mode_select_callback(self, request, response):
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
            self.get_logger().info("Now is teleop reference velocity from end effector mode")
            return response
        elif request.mode == 3:
            self.mode = 3
            response.success = True
            self.get_logger().info("Success to change mode")
            self.get_logger().info("Now is teleop reference velocity from world mode")
            return response
        elif request.mode == 4:
            self.mode = 4
            response.success = True
            self.get_logger().info("Success to change mode")
            self.get_logger().info("Now is auto mode")
            return response
        
    def timer_callback(self):
        try:
            if self.mode == 0 :
                # self.joint_target =  self.joint_current
                self.publish_joint_state(self.joint_current)
            elif self.mode == 1 :
                # self.get_logger().info(f'current joint angle are :{self.joint_current}')
                # for i in range(3):
                    # self.joint_current[i], self.trajc[i] = self.trajcompute(self.joint_current[i], self.joint_target[i], self.trajp[i], 0.000001, self.trajc[i], i)
                self.joint_current[0], self.trajc[0] = self.trajcompute(self.joint_current[0], self.joint_target[0], self.trajp[0], 0.000001, self.trajc[0], 0)
                self.joint_current[1], self.trajc[1] = self.trajcompute(self.joint_current[1], self.joint_target[1], self.trajp[1], 0.000001, self.trajc[1], 1)
                self.joint_current[2], self.trajc[2] = self.trajcompute(self.joint_current[2], self.joint_target[2], self.trajp[2], 0.000001, self.trajc[2], 2)
                # self.get_logger().info(f"The q path are {self.trajp}")
                # self.get_logger().info(f"The q current position are {self.joint_current}")
                # self.get_logger().info(f"The q target position are {self.joint_target}")
                self.publish_joint_state(self.joint_current)
            elif self.mode == 2:
                self.get_logger().info("Teleop mode")
            elif self.mode == 3:
                self.get_logger().info("Auto mode")
            elif self.mode == 4:
                pass
                # if self.trajisfinish == 0:
                #     for i in range(3):
                #         self.joint_current[i], self.trajc[i] = self.trajcompute(self.joint_current[i], self.joint_target[i], self.trajp[i], 0.000001, self.trajc[i], i)
                #         self.publish_joint_state(self.joint_current)
                # elif self.trajisfinish == 1:
                #     request = SetModePosition.Request()
                #     future = self.random_client.call_async(request)
                #     future.add_done_callback(self.get_random)   
                #     self.trajisfinish = 2
                #     self.publish_joint_state(self.joint_current)
                # else:
                #     self.publish_joint_state(self.joint_current)
            
        except Exception as e:
            self.get_logger().error(f"SchedulerNode has {e}")
    
    def get_random(self, future):
        response = future.result()
        # self.get_logger().info(f'{response}')
        request = SetModePosition.Request()
        request.position.x = response.position.x
        request.position.y = response.position.y
        request.position.z = response.position.z
        self.ik_target_client.call_async(request)
        future.add_done_callback(self.send_ik_target)  
        
    def send_ik_target(self, future):
        response = future.result()
        if response.success == False:
            self.get_logger().error(f"Can not receive target!. Reset mode")
            self.mode = 0
         
    def ik_config_space_callback(self, request, response):
        for i in range(3):
            self.trajc[i] = 0
        self.joint_target[0] = request.position.x
        self.joint_target[1] = request.position.y
        self.joint_target[2] = request.position.z
        self.get_logger().info(f'target joint angle are :{self.joint_target}')
        for i in range(3):
            self.trajp[i] = self.normalize_angle(math.fabs(self.joint_current[i] - self.joint_target[i]))     
        # self.trajisfinish = 0
        response.success = True
        return response
    
    def trajcompute(self, q, target, path, tol, count, i):
        times = int(self.trajtime * self.rate)
        self.get_logger().info(f"The q is {q} target is {target}, is equal {q == target}")
        # self.get_logger().info(f"The q>target is {q>target}")
        # self.get_logger().info(f"The q<target is {q<target}")
        if math.fabs(q - target) <= tol or count == times:
            # self.get_logger().info(f"Joint_{i+1} was arrived")
            # self.trajisfinish = 1
            return q, count
        elif q > target:
            count += 1
            self.get_logger().info(f"The q is {q+(path/times)}")
            return (q-(path/times)), count
        elif q < target:
            count += 1
            self.get_logger().info(f"The q is {q+(path/times)}")
            # self.get_logger().info(f"The q path are {self.trajp}")
            return (q+(path/times)), count

    def normalize_angle(self, angle):
        """Normalize angle to the range [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = q
        msg.name = self.joint_name
        self.taskspace_target_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
