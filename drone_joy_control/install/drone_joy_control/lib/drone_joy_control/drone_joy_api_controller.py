#!/usr/bin/env python3

"""
Drone joystick controller using Aerostack2 Python API
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from as2_python_api.drone_interface import DroneInterface
from threading import Thread
import time
import signal

class DroneJoyController(Node):
    def __init__(self):
        super().__init__('drone_joy_api_controller')
        
        # Parameters
        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 0.8)  # rad/s
        self.declare_parameter('max_yaw_speed', 1.0)  # rad/s
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('takeoff_height', 1.0)  # meters
        
        # Get parameters
        self.drone_id = self.get_parameter('drone_id').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_yaw_speed = self.get_parameter('max_yaw_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        
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
        
        # Initialize the drone interface
        self.get_logger().info(f"Initializing drone interface for {self.drone_id}")
        self.drone = DroneInterface(self.drone_id, verbose=True, use_sim_time=True)
        
        # Create Joy subscriber with a higher QoS profile
        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            joy_qos
        )
        
        # Create a higher frequency timer for control updates (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
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
        # Don't call land_drone here, let the main loop handle it
        
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale value to be from 0 to 1 after deadzone
        sign = 1.0 if value > 0 else -1.0
        normalized = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * normalized
    
    def joy_callback(self, msg):
        """Store joystick data for processing in control loop."""
        # Update joystick data and timestamp
        self.joy_data = msg
        self.last_joy_timestamp = time.time()
        self.get_logger().debug(f"Joy update: Axes: {msg.axes}, Buttons: {msg.buttons}")
    
    def check_joy_connection(self):
        """Check if joystick is still connected and sending data."""
        elapsed = time.time() - self.last_joy_timestamp
        if elapsed > 5.0:
            self.get_logger().warn(f"No joystick data received for {elapsed:.1f} seconds!")
        else:
            self.get_logger().debug(f"Joystick connected. Last data: {elapsed:.1f}s ago")
    
    def control_loop(self):
        """Process joystick inputs and control the drone."""
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
        
        # Debug log to verify we're processing input
        self.get_logger().debug(f"Processing joystick: flying={self.is_flying}, axes={len(self.joy_data.axes)}, buttons={len(self.joy_data.buttons)}")
        
        # Button actions (check array bounds first)
        if self.BTN_TAKEOFF < buttons_len and self.joy_data.buttons[self.BTN_TAKEOFF] == 1 and not self.is_flying:
            self.start_drone()
        
        elif self.BTN_LAND < buttons_len and self.joy_data.buttons[self.BTN_LAND] == 1 and self.is_flying:
            self.land_drone()
        
        # Process movement if flying
        elif self.is_flying and all(idx < axes_len for idx in [self.AXIS_ROLL, self.AXIS_PITCH, self.AXIS_YAW, self.AXIS_ALTITUDE]):
            # Get joystick inputs with deadzone
            roll = self.apply_deadzone(self.joy_data.axes[self.AXIS_ROLL])
            pitch = self.apply_deadzone(self.joy_data.axes[self.AXIS_PITCH])
            yaw = self.apply_deadzone(self.joy_data.axes[self.AXIS_YAW])
            altitude = self.apply_deadzone(self.joy_data.axes[self.AXIS_ALTITUDE])
            
            # Check if any control input is active
            if any([abs(roll) > 0, abs(pitch) > 0, abs(yaw) > 0, abs(altitude) > 0]):
                self.last_movement_time = time.time()
                
                # Convert joystick values to actual speeds
                vx = pitch * self.max_linear_speed  # Forward/backward
                vy = -roll * self.max_linear_speed  # Left/right
                vz = altitude * self.max_linear_speed  # Up/down
                vyaw = -yaw * self.max_yaw_speed    # Yaw rotation
                
                # Send speed commands to drone
                self.drone.speed_control.send_speed_command(
                    vx=vx, 
                    vy=vy, 
                    vz=vz, 
                    vyaw=vyaw
                )
                
                self.get_logger().info(f"Speed cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, vyaw={vyaw:.2f}")
            
            # If no movement for a while, hover
            elif time.time() - self.last_movement_time > 0.5:
                # Send zero speeds to hover
                self.drone.speed_control.send_speed_command(
                    vx=0.0, 
                    vy=0.0, 
                    vz=0.0, 
                    vyaw=0.0
                )
                self.get_logger().debug("Hovering - no joystick movement")
    
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