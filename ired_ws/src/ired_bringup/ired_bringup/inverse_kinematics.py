#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from ired_msgs.msg import MotorSpeed

class INVERSEKINEMATICS(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()
        
        # Initialise ROS Publisher and Subscriber
        self.speed_control_pub = self.create_publisher(MotorSpeed, self.inverse_kinematics_topic, 10)
        self.create_subscription(TwistStamped, self.cmd_vel_topic, self.fetchCommandVelocityCallback, 10)
        
        # Initialise ROS Timers
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.update_callback)
        
        # ROS Logger
        self.get_logger().info(f"Inverse : ROS publisher on {self.inverse_kinematics_topic} [ired_msgs/MotorSpeed]")
        self.get_logger().info(f"Inverse : ROS subscriber on {self.cmd_vel_topic} [geometry_msgs/TwistStamped]")
    
    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_control_topic', '/ired/motor/speed_control')
        self.declare_parameter('wheel_seperation', 0.199)
        self.declare_parameter('mps_to_rpm', 280.8616642798)
        
        # ROS Parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.inverse_kinematics_topic = self.get_parameter('motor_control_topic').get_parameter_value().string_value
        self.wheel_seperation = self.get_parameter('wheel_seperation').get_parameter_value().double_value
        self.mps_to_rpm = self.get_parameter('mps_to_rpm').get_parameter_value().double_value

        self.last_time_speed_fb = self.get_clock().now()
    
    def initial_variables(self):
        self.goal_linear_velocity_x = 0.0
        self.goal_linear_velocity_y = 0.0
        self.goal_angular_velocity_z = 0.0
        self.wheel_speed_cmd = [0.0, 0.0, 0.0, 0.0]
        self.motor_speed = MotorSpeed()
        self.motor_speed.speed = [0.0, 0.0, 0.0, 0.0]
        self.FLEFT = 0
        self.FRIGHT = 1
        self.RLEFT = 2
        self.RRIGHT = 3
        self.last_time_cmd_vel = self.get_clock().now()
        
    def fetchCommandVelocityCallback(self, msg):
        self.last_time_cmd_vel = self.get_clock().now()
        
        self.goal_linear_velocity_x = msg.twist.linear.x
        self.goal_linear_velocity_y = msg.twist.linear.y
        self.goal_angular_velocity_z = msg.twist.angular.z
        
    def calculateInverseKinematics(self):
        # Edit code here
        self.wheel_speed_cmd[self.FLEFT] = -1.0 * (self.goal_linear_velocity_x - (self.goal_angular_velocity_z * self.wheel_seperation) / 2)
        self.wheel_speed_cmd[self.FRIGHT] = self.goal_linear_velocity_x + (self.goal_angular_velocity_z * self.wheel_seperation) / 2
        self.wheel_speed_cmd[self.RLEFT] = 0.0
        self.wheel_speed_cmd[self.RRIGHT] = 0.0

        self.motor_speed.speed[self.FLEFT] = self.wheel_speed_cmd[self.FLEFT] * self.mps_to_rpm
        self.motor_speed.speed[self.FRIGHT] = self.wheel_speed_cmd[self.FRIGHT] * self.mps_to_rpm
        self.motor_speed.speed[self.RLEFT] = self.wheel_speed_cmd[self.RLEFT] * self.mps_to_rpm
        self.motor_speed.speed[self.RRIGHT] = self.wheel_speed_cmd[self.RRIGHT] * self.mps_to_rpm


    def update_callback(self):
        time_now = self.get_clock().now()
        
        if (time_now - self.last_time_cmd_vel).nanoseconds / 1e9 > 1.0:
            self.goal_linear_velocity_x = 0.0
            self.goal_linear_velocity_y = 0.0
            self.goal_angular_velocity_z = 0.0
        
        self.calculateInverseKinematics()
        self.speed_control_pub.publish(self.motor_speed)

def main(args=None):
    rclpy.init(args=args)
    inverse_kinematics = INVERSEKINEMATICS()
    
    rclpy.spin(inverse_kinematics)
    
    inverse_kinematics.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
