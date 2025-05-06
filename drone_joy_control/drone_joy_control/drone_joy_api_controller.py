#!/usr/bin/env python3

"""
Drone joystick controller using Aerostack2 Python API
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop # for teleop mode
from threading import Thread, Lock
import time
import signal
import numpy as np
import math

class DroneJoyController(Node):
    def __init__(self):
        super().__init__('drone_joy_api_controller')
        
        # Parameters
        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 0.8)  # rad/s
        self.declare_parameter('max_yaw_speed', 1.0)  # rad/s
        self.declare_parameter('deadzone', 0.01)
        self.declare_parameter('takeoff_height', 1.0)  # meters
        self.declare_parameter('control_distance', 0.2)  # meters to move per joystick input (reduced for smoother control)
        self.declare_parameter('control_rate', 0.02)  # seconds between control updates (20 Hz)
        self.declare_parameter('joy_max_value', 2.7)  # Maximum expected value from joy (default 1.0, could be 255)
        self.declare_parameter('joy_scale_factor', 0.1)  # Scale factor to reduce movement intensity
        
        # Get parameters
        self.drone_id = self.get_parameter('drone_id').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_yaw_speed = self.get_parameter('max_yaw_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.control_distance = self.get_parameter('control_distance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.joy_max_value = self.get_parameter('joy_max_value').value
        self.joy_scale_factor = self.get_parameter('joy_scale_factor').value
        
        # Button mapping (configurable)
        self.declare_parameter('btn_takeoff', 0)  # A button
        self.declare_parameter('btn_land', 1)     # B button
        self.declare_parameter('axis_roll', 0)    # Left stick left/right
        self.declare_parameter('axis_pitch', 1)   # Left stick up/down
        self.declare_parameter('axis_yaw', 2)     # Right stick left/right
        self.declare_parameter('axis_altitude', 3)  # Right stick up/down
        
        # Get button mappings
        self.BTN_TAKEOFF = self.get_parameter('btn_takeoff').value
        self.BTN_LAND = self.get_parameter('btn_land').value
        self.AXIS_ROLL = self.get_parameter('axis_roll').value
        self.AXIS_PITCH = self.get_parameter('axis_pitch').value
        self.AXIS_YAW = self.get_parameter('axis_yaw').value
        self.AXIS_ALTITUDE = self.get_parameter('axis_altitude').value
        
        # Status variables
        self.is_flying = False
        self.is_armed = False
        self.joy_data = None
        self.last_joy_timestamp = time.time()
        self.last_movement_time = time.time()
        self.shutdown_requested = False
        self.movement_in_progress = False
        self.lock = Lock()  # For thread safety
        self.target_position = None  # Target position to move to
        self.last_sent_position = None  # Last position sent to the drone
        
        # Initialize current pose
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.z = 0.0
        
        # Setup QoS profile for position control
        position_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Initialize the drone interface
        self.get_logger().info(f"Initializing drone interface for {self.drone_id}")
        self.drone = DroneInterface(self.drone_id, verbose=True, use_sim_time=True)
        self.drone_teleop = DroneInterfaceTeleop(self.drone_id, verbose=True, use_sim_time=True)
        
        # Create Joy subscriber with a higher QoS profile
        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            joy_qos
        )
        
        # Subscribe to drone position
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_id}/self_localization/pose',
            self.pose_callback,
            position_qos
        )
        
        # Create a timer for processing joystick input
        self.control_timer = self.create_timer(self.control_rate, self.control_loop)
        
        # Create a separate timer for sending movement commands at a controlled rate
        self.movement_timer = self.create_timer(0.1, self.movement_loop)
        
        # Heartbeat timer to check joystick connection
        self.heartbeat_timer = self.create_timer(1.0, self.check_joy_connection)
        
        # Register signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info(f'Drone API Joy Controller initialized for {self.drone_id}')
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals properly"""
        self.get_logger().info(f"Received signal {sig}, initiating clean shutdown")
        self.shutdown_requested = True
    
    def normalize_joy_input(self, value):
        """Normalize joystick input to range -1.0 to 1.0."""
        # First normalize to -1.0 to 1.0 based on the max value
        if self.joy_max_value != 1.0:
            value = value / self.joy_max_value
        
        # Then apply the scale factor to reduce movement intensity
        value = value * self.joy_scale_factor
        
        return value
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input."""
        # First normalize the input
        print(f"Value before normalization: {value}")
        #value = self.normalize_joy_input(value)
        print(f"Value after normalization: {value}")
        
        # Then apply deadzone
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale value to be from 0 to 1 after deadzone
        sign = 1.0 if value > 0 else -1.0
        normalized = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * normalized
    
    def pose_callback(self, msg):
        """Store current drone pose."""
        self.current_pose = msg
        self.get_logger().debug(f"Current position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}")
    
    def joy_callback(self, msg):
        """Store joystick data for processing in control loop."""
        # Check if we need to log the raw values to help with debugging
        if len(msg.axes) > 0:
            raw_values = [f"{v:.2f}" for v in msg.axes]
            self.get_logger().debug(f"Raw joy values: [{', '.join(raw_values)}]")
        
        # Update joystick data and timestamp
        self.joy_data = msg
        self.last_joy_timestamp = time.time()
    
    def check_joy_connection(self):
        """Check if joystick is still connected and sending data."""
        elapsed = time.time() - self.last_joy_timestamp
        if elapsed > 5.0:
            self.get_logger().warn(f"No joystick data received for {elapsed:.1f} seconds!")
        else:
            self.get_logger().debug(f"Joystick connected. Last data: {elapsed:.1f}s ago")
    
    def get_current_position(self):
        """Get the current position from the drone state."""
        # Extract current position from the PoseStamped message
        pos = [
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ]
        
        # Calculate yaw from the quaternion
        current_q = self.current_pose.pose.orientation
        yaw = math.atan2(
            2.0 * (current_q.w * current_q.z + current_q.x * current_q.y),
            1.0 - 2.0 * (current_q.y * current_q.y + current_q.z * current_q.z)
        )
        
        return pos, yaw
    
    def update_position_from_joy(self, roll, pitch, yaw, altitude):
        """Calculate new position based on joystick input."""
        # Get current position and yaw
        pos, current_yaw = self.get_current_position()
        
        # Calculate movement distances based on joystick input
        # These values are already normalized and scaled by apply_deadzone
        dx = pitch * self.control_distance
        dy = -roll * self.control_distance
        dz = altitude * self.control_distance
        dyaw = -yaw * np.pi/10  # Reduced rotation amount
        
        # Calculate new position in local frame
        new_x = pos[0] + dx
        new_y = pos[1] + dy
        new_z = pos[2] + dz
        new_yaw = current_yaw + dyaw
        
        # Make sure we don't go below ground level
        if new_z < 0.1:
            new_z = 0.1
        
        # Return updated position
        new_position = [new_x, new_y, new_z]
        
        return new_position, new_yaw
    
    def control_loop(self):
        """Process joystick inputs and update target position."""
        # Handle shutdown request
        if self.shutdown_requested and self.is_flying:
            self.get_logger().info("Shutdown requested: Landing drone")
            self.land_drone()
            return
            
        # Skip if no joystick data available
        if not self.joy_data:
            self.get_logger().debug("No joystick data available")
            return
            
        # Check if the axes array has enough elements to safely access our indices
        axes_len = len(self.joy_data.axes)
        buttons_len = len(self.joy_data.buttons)
        
        # Button actions (check array bounds first)
        if self.BTN_TAKEOFF < buttons_len and self.joy_data.buttons[self.BTN_TAKEOFF] == 1 and not self.is_flying:
            self.start_drone()
            return
        
        elif self.BTN_LAND < buttons_len and self.joy_data.buttons[self.BTN_LAND] == 1 and self.is_flying:
            self.land_drone()
            return
        
        # Process movement if flying
        if self.is_flying and all(idx < axes_len for idx in [self.AXIS_ROLL, self.AXIS_PITCH, self.AXIS_YAW, self.AXIS_ALTITUDE]):
            # Get joystick inputs with deadzone and normalization
            roll = self.apply_deadzone(self.joy_data.axes[self.AXIS_ROLL])
            pitch = self.apply_deadzone(self.joy_data.axes[self.AXIS_PITCH])
            yaw = self.apply_deadzone(self.joy_data.axes[self.AXIS_YAW])
            altitude = self.apply_deadzone(self.joy_data.axes[self.AXIS_ALTITUDE])
            
            # Log the normalized values
            self.get_logger().debug(f"Normalized inputs: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}, alt={altitude:.3f}")
            
            # Check if any control input is active (threshold reduced for smoother response)
            if any([abs(roll) > 0.01, abs(pitch) > 0.01, abs(yaw) > 0.01, abs(altitude) > 0.01]):
                self.last_movement_time = time.time()
                
                # Calculate new position based on joystick input
                new_position, new_yaw = self.update_position_from_joy(roll, pitch, yaw, altitude)
                
                # Update target position with thread safety
                with self.lock:
                    self.target_position = (new_position, new_yaw)
                
                # Log the calculated target
                self.get_logger().debug(f"New target: x={new_position[0]:.2f}, y={new_position[1]:.2f}, z={new_position[2]:.2f}, yaw={new_yaw:.2f}")
    
    def movement_loop(self):
        """Send movement commands at a controlled rate."""
        # Skip if not flying or no target
        if not self.is_flying or self.target_position is None:
            return
        
        # Get the current target with thread safety
        with self.lock:
            position, yaw = self.target_position
            
        # Check if this is a new position (with some tolerance to avoid jitter)
        if self.last_sent_position is None or any(abs(a - b) > 0.01 for a, b in zip(self.last_sent_position, position)):
            self.get_logger().info(f"Moving to: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")
            
            try:

                # Create a PoseStamped message from position and yaw
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "earth"  # Use appropriate frame
                pose_msg.pose.position.x = position[0]
                pose_msg.pose.position.y = position[1]
                pose_msg.pose.position.z = position[2]
                
                # Convert yaw to quaternion
                cy = np.cos(yaw * 0.5)
                sy = np.sin(yaw * 0.5)
                pose_msg.pose.orientation.w = cy
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = sy
                

                # Send the position command with yaw angle using PoseStamped
                self.drone_teleop.motion_ref_handler.position.send_position_command_with_yaw_angle(pose=pose_msg, yaw_angle=yaw)
                
                # Update last sent position
                self.last_sent_position = position
            except Exception as e:
                self.get_logger().error(f"Error sending movement command: {str(e)}")
    
    def start_drone(self):
        """Execute offboard, arm and takeoff sequence."""
        try:
            self.get_logger().info("Starting preflight checks")
            # Go to offboard mode
            self.drone.offboard()
            
            # Arm the drone
            self.drone.arm()
            self.is_armed = True
            
            # Takeoff
            self.get_logger().info(f"Taking off to {self.takeoff_height}m")
            self.drone.takeoff(self.takeoff_height, speed=1.0)
            self.is_flying = True
            
            self.get_logger().info("Takeoff complete. Ready for joystick control.")
        except Exception as e:
            self.get_logger().error(f"Error during takeoff: {str(e)}")
    
    def land_drone(self):
        """Execute landing sequence."""
        try:
            self.get_logger().info("Landing")
            self.drone.land(speed=0.5)
            self.is_flying = False
            
            # Disarm after landing
            time.sleep(2.0)  # Wait for landing to complete
            self.drone.disarm()
            self.is_armed = False
            
            self.get_logger().info("Landing complete")
        except Exception as e:
            self.get_logger().error(f"Error during landing: {str(e)}")

# Global variable to track the controller node instance
controller_node = None

def main(args=None):
    global controller_node
    
    rclpy.init(args=args)
    controller_node = DroneJoyController()
    
    try:
        # Set logging level to INFO
        rclpy.logging.set_logger_level('drone_joy_api_controller', rclpy.logging.LoggingSeverity.INFO)
        
        # Start ROS2 spin in a separate thread to avoid blocking
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(controller_node)
        executor_thread = Thread(target=executor.spin)
        executor_thread.daemon = True
        executor_thread.start()
        
        # Keep the main thread alive - no shutdown check here
        # because we want to keep running until explicit termination
        while rclpy.ok():
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        controller_node.get_logger().info("KeyboardInterrupt caught")
    except Exception as e:
        if controller_node:
            controller_node.get_logger().error(f"Exception: {str(e)}")
    finally:
        # We should NOT automatically land when the script ends
        # Only land if specifically requested through the shutdown flag
        if controller_node and controller_node.is_flying and controller_node.shutdown_requested:
            controller_node.get_logger().info("Regular shutdown: Landing drone")
            controller_node.land_drone()
        
        if controller_node:
            controller_node.get_logger().info("Shutting down node")
            controller_node.drone.shutdown()
            controller_node.destroy_node()
        
        # Clear the global reference
        controller_node = None
        rclpy.shutdown()

if __name__ == '__main__':
    main() 