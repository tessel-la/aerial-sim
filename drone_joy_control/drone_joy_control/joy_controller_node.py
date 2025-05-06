#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
from as2_msgs.msg import PoseWithID
from std_msgs.msg import Header
import math
import numpy as np

class JoyControllerNode(Node):
    def __init__(self):
        super().__init__('joy_controller_node')
        
        # Parameters
        self.declare_parameter('drone_id', 'drone_sim_0')
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 0.8)  # rad/s
        self.declare_parameter('max_yaw_speed', 1.0)  # rad/s
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('takeoff_height', 1.0)  # meters
        
        self.drone_id = self.get_parameter('drone_id').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_yaw_speed = self.get_parameter('max_yaw_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        
        # Status variables
        self.is_flying = False
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.z = 0.0
        
        # Button mapping
        self.BTN_A = 0  # Takeoff
        self.BTN_B = 1  # Land
        self.BTN_X = 2  # Emergency stop
        self.AXES_LEFT_LR = 0  # Left stick left/right (roll)
        self.AXES_LEFT_UD = 1  # Left stick up/down (pitch)
        self.AXES_RIGHT_LR = 2  # Right stick left/right (yaw)
        self.AXES_RIGHT_UD = 3  # Right stick up/down (altitude)
        
        # Setup QoS profile for position control
        position_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_id}/self_localization/pose',
            self.pose_callback,
            position_qos
        )
        
        # Publishers
        self.position_setpoint_pub = self.create_publisher(
            PoseWithID,
            f'/{self.drone_id}/motion_reference/position',
            position_qos
        )
        
        self.get_logger().info(f'Drone Joystick Controller initialized for {self.drone_id}')
        
    def pose_callback(self, msg):
        """Store current drone pose."""
        self.current_pose = msg
        
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale value to be from 0 to 1 after deadzone
        sign = 1.0 if value > 0 else -1.0
        normalized = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * normalized
        
    def joy_callback(self, msg):
        """Process joystick inputs."""
        # Button actions
        if msg.buttons[self.BTN_A] == 1 and not self.is_flying:
            self.takeoff()
        
        if msg.buttons[self.BTN_B] == 1 and self.is_flying:
            self.land()
            

        # Only process movement if flying
        if self.is_flying:
            # Get joystick inputs with deadzone
            roll = self.apply_deadzone(msg.axes[self.AXES_LEFT_LR])
            pitch = self.apply_deadzone(msg.axes[self.AXES_LEFT_UD])
            yaw = self.apply_deadzone(msg.axes[self.AXES_RIGHT_LR])
            altitude = self.apply_deadzone(msg.axes[self.AXES_RIGHT_UD])
            
            # Create position setpoint based on current position and joystick inputs
            setpoint = PoseWithID()
            setpoint.id = self.drone_id
            
            # Get current pose from state
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            current_z = self.current_pose.pose.position.z
            
            # Calculate new position setpoint
            # Moving left/right (roll)
            setpoint.pose.position.x = current_x + roll * self.max_linear_speed * 0.1
            
            # Moving forward/backward (pitch)
            setpoint.pose.position.y = current_y + pitch * self.max_linear_speed * 0.1
            
            # Moving up/down (altitude)
            setpoint.pose.position.z = current_z + altitude * self.max_linear_speed * 0.1
            
            # Ensure minimum altitude
            if setpoint.pose.position.z < 0.1:
                setpoint.pose.position.z = 0.1
                
            # Yaw control based on right stick
            # Calculate yaw from current orientation quaternion
            current_q = self.current_pose.pose.orientation
            yaw_rad = math.atan2(
                2.0 * (current_q.w * current_q.z + current_q.x * current_q.y),
                1.0 - 2.0 * (current_q.y * current_q.y + current_q.z * current_q.z)
            )
            
            # Update yaw
            new_yaw = yaw_rad + yaw * self.max_yaw_speed * 0.1
            
            # Convert back to quaternion
            qz = math.sin(new_yaw / 2.0)
            qw = math.cos(new_yaw / 2.0)
            
            # Set orientation (simplified quaternion for yaw-only rotation)
            setpoint.pose.orientation.x = 0.0
            setpoint.pose.orientation.y = 0.0
            setpoint.pose.orientation.z = qz
            setpoint.pose.orientation.w = qw
            
            # Publish setpoint
            self.position_setpoint_pub.publish(setpoint)
    
    def takeoff(self):
        """Execute takeoff sequence."""
        self.get_logger().info(f"Taking off to {self.takeoff_height}m")
        
        # Set takeoff position
        takeoff_pose = PoseWithID()
        takeoff_pose.id = self.drone_id
        
        # Get current position
        takeoff_pose.pose.position.x = self.current_pose.pose.position.x
        takeoff_pose.pose.position.y = self.current_pose.pose.position.y
        takeoff_pose.pose.position.z = self.takeoff_height
        
        # Keep current orientation
        takeoff_pose.pose.orientation = self.current_pose.pose.orientation
        
        # Publish takeoff position
        self.position_setpoint_pub.publish(takeoff_pose)
        self.is_flying = True
        
    def land(self):
        """Execute landing sequence."""
        self.get_logger().info("Landing")
        
        # Set landing position
        landing_pose = PoseWithID()
        landing_pose.id = self.drone_id
        
        # Get current position but set z to 0
        landing_pose.pose.position.x = self.current_pose.pose.position.x
        landing_pose.pose.position.y = self.current_pose.pose.position.y
        landing_pose.pose.position.z = 0.0
        
        # Keep current orientation
        landing_pose.pose.orientation = self.current_pose.pose.orientation
        
        # Publish landing position
        self.position_setpoint_pub.publish(landing_pose)
        self.is_flying = False
        
    def emergency_stop(self):
        """Execute emergency stop."""
        self.get_logger().info("EMERGENCY STOP")
        # Logic for emergency stop would go here
        # This would depend on the specific API available
        self.is_flying = False

def main(args=None):
    rclpy.init(args=args)
    node = JoyControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 