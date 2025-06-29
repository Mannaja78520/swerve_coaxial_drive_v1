#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rclpy import qos
from src.utilize import * 
from src.controller import *
import math
from typing import List, Tuple


class robot_movement(Node):

    def __init__(self):
        super().__init__("robot_movement")

        # Declare parameters with default values
        self.declare_parameter("movement_mode", "mps")
        self.declare_parameter("max_pwm_Speed", 1023.0)
        self.declare_parameter("max_rpm", 6000)
        self.declare_parameter("max_linear_speed", 10.0)
        self.declare_parameter("wheel_diameter", 0.762)
        self.declare_parameter("gear_ratio", 18.0)
        self.declare_parameter("wheel_distance", 0.3)  # L

        # Load parameters
        self.movement_mode: str = self.get_parameter("movement_mode").get_parameter_value().string_value
        self.maxPWMSpeed: float = self.get_parameter("max_pwm_Speed").get_parameter_value().double_value
        self.maxRPM: int = self.get_parameter("max_rpm").get_parameter_value().integer_value
        self.max_linear_speed: float = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.wheel_diameter: float = self.get_parameter("wheel_diameter").get_parameter_value().double_value
        self.gear_ratio: float = self.get_parameter("gear_ratio").get_parameter_value().double_value
        L: float = self.get_parameter("wheel_distance").get_parameter_value().double_value
        
        self.safe_maxRPM: float = self.maxRPM * 0.85
        
        self.wheel_commands: List[Tuple[float, float]] = [(0.0, 0.0)] * 3

        # Wheel location relative to robot center (in meters)
        # Equilateral triangle layout (L = distance from center to wheel)
        L = 0.3  # Change this based on your actual robot dimensions
        self.wheel_positions = [
            (L, 0),                          # Wheel 1: front
            (-L/2, math.sqrt(3)*L/2),        # Wheel 2: rear-left
            (-L/2, -math.sqrt(3)*L/2)        # Wheel 3: rear-right
        ]


 
        self.send_robot_move_speed = self.create_publisher(
            Twist, "/wheel/motor_speed", qos_profile=qos.qos_profile_system_default
        )
        
        self.send_robot_wheel_angle = self.create_publisher(
            Twist, "/wheel/angle", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_sub, qos_profile=qos.qos_profile_sensor_data
        )
        
        self.create_subscription(
            String, '/movement_mode', self.movement_mode_sub, qos_profile=qos.qos_profile_sensor_data
        )


        self.sent_data_timer = self.create_timer(0.03, self.sendData)
        
    def swerve_inverse_kinematics(self, Vx: float, Vy: float, omega: float) -> List[Tuple[float, float]]:
        """
        Calculate speed and angle for each wheel in a 3-wheel swerve drive.
        
        :param Vx: Linear velocity in the X direction (forward), in m/s
        :param Vy: Linear velocity in the Y direction (left), in m/s
        :param omega: Angular velocity around Z axis (yaw), in rad/s
        :return: List of (speed, angle) for each wheel
        """
        results = []

        for idx, (x, y) in enumerate(self.wheel_positions):
            # Calculate local velocity vector at each wheel
            vix = Vx - omega * y
            viy = Vy + omega * x

            # Calculate speed and angle
            speed = math.hypot(vix, viy)
            angle = math.atan2(viy, vix)  # in radians

            results.append((speed, angle))

        return results    
    
    def cmd_vel_sub(self, msg):
        self.moveSpeed = msg.linear.x 
        self.slideSpeed = msg.linear.y
        self.turnSpeed = msg.angular.z 
        
        DEAD_ZONE = 0.01
        if abs(self.moveSpeed) < DEAD_ZONE:
            self.moveSpeed = 0.0
        if abs(self.slideSpeed) < DEAD_ZONE:
            self.slideSpeed = 0.0
        if abs(self.turnSpeed) < DEAD_ZONE:
            self.turnSpeed = 0.0
        
        # Vx, Vy, omega are the linear and angular velocities
        self.wheel_commands = self.swerve_inverse_kinematics(self.moveSpeed, self.slideSpeed, self.turnSpeed)
        
        
        max_speed: float = self.max_linear_speed if self.movement_mode == "mps" else self.maxPWMSpeed
        # Normalize wheel speeds to ensure they fit within the max speed
        D = max(abs(self.moveSpeed) + abs(self.slideSpeed) + abs(self.turnSpeed), max_speed)
        # Scale wheel speeds by D if needed
        self.wheel_commands = [
            (speed / D * max_speed, angle)
            for (speed, angle) in self.wheel_commands
        ]
        #use mps
        if self.movement_mode == "mps":
            print("[mps] mode")
            for i, (speed, angle) in enumerate(self.wheel_commands):
                print(f"Wheel {i+1}: speed = {speed:.2f} m/s, angle = {math.degrees(angle):.2f}°")
            
        #use pwm
        elif self.movement_mode == "manual":
            print("[manual] mode (PWM input, angle estimated)")
            for i, (pwm, angle) in enumerate(self.wheel_commands):
                print(f"  Wheel {i+1}: speed = {pwm:.0f} PWM, angle = {math.degrees(angle):.1f}°")


            
            
            
    def movement_mode_sub(self, msg):
        self.movement_mode = msg.data
        self.get_logger().info(f"Movement mode set to: {self.movement_mode}")
        
        
    def sendData(self):
        send_robot_move_speed_msg = Twist()
        robot_wheel_angle_msg = Twist()
       
        def mps_to_rpm(speed_mps):
            return (speed_mps * 60) / (math.pi * self.wheel_diameter)

        def normalize_rpms(rpms: List[float]) -> List[float]:
            max_rpm = max(abs(r) for r in rpms)
            if max_rpm > self.safe_maxRPM:
                scale = self.safe_maxRPM / max_rpm
                return [r * scale for r in rpms]
            return rpms

        if self.movement_mode == "mps":
            # Convert each wheel speed from m/s to RPM
            rpm_values = [mps_to_rpm(speed) * self.gear_ratio for speed, _ in self.wheel_commands]
            rpm_values = normalize_rpms(rpm_values)

            send_robot_move_speed_msg.linear.x = float(rpm_values[0])
            send_robot_move_speed_msg.linear.y = float(rpm_values[1])
            send_robot_move_speed_msg.linear.z = float(rpm_values[2])
            
        elif self.movement_mode == "manual":
            send_robot_move_speed_msg.linear.x = float(self.wheel_commands[0][0])
            send_robot_move_speed_msg.linear.y = float(self.wheel_commands[1][0])
            send_robot_move_speed_msg.linear.z = float(self.wheel_commands[2][0])

        robot_wheel_angle_msg.linear.x = float(math.degrees(self.wheel_commands[0][1]))
        robot_wheel_angle_msg.linear.y = float(math.degrees(self.wheel_commands[1][1]))
        robot_wheel_angle_msg.linear.z = float(math.degrees(self.wheel_commands[2][1]))

        self.send_robot_move_speed.publish(send_robot_move_speed_msg)
        self.send_robot_wheel_angle.publish(robot_wheel_angle_msg)


def main():
    rclpy.init()

    sub = robot_movement()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()