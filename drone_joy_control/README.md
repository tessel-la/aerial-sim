# Drone Joy Control Package

This ROS2 package allows control of Aerostack2 drones using a joystick/gamepad controller. It subscribes to joy topics and uses the Aerostack2 position control API to control the drone.

## Prerequisites

- ROS2 Humble
- A connected joystick controller (Xbox, PlayStation, etc.)
- Aerostack2 framework

## Installation

The package should be placed in your Aerostack2 workspace source directory. The package has been configured to work with the Aerostack2 simulator docker container.

## Usage

### Using Tmuxinator

The package has been integrated into the Aerostack2 tmuxinator configuration. To start everything including the joystick controller:

```bash
cd simulation
tmuxinator start -p aerostack2.yml
```

### Manual Launch

You can also run just the joystick controller using the provided script:

```bash
cd simulation
./run_joy_control.sh
```

## Controller Mappings

The default controller mappings are:

- **A button (0)**: Takeoff
- **B button (1)**: Land
- **X button (2)**: Emergency stop
- **Left stick left/right**: Roll (move left/right)
- **Left stick up/down**: Pitch (move forward/backward)
- **Right stick left/right**: Yaw rotation
- **Right stick up/down**: Altitude control

You can modify these mappings in the `config/joy_config.yaml` file.

## Configuration

The controller settings can be customized by editing the `config/joy_config.yaml` file:

```yaml
/**:
  ros__parameters:
    # Controller button mappings
    btn_takeoff: 0     # A button
    btn_land: 1        # B button
    btn_emergency: 2   # X button
    
    # Joystick axis mappings
    axis_roll: 0       # Left stick left/right
    axis_pitch: 1      # Left stick up/down
    axis_yaw: 3        # Right stick left/right
    axis_altitude: 4   # Right stick up/down
    
    # Control parameters
    max_linear_speed: 1.0  # m/s
    max_angular_speed: 0.8  # rad/s
    max_yaw_speed: 1.0  # rad/s
    deadzone: 0.1
    takeoff_height: 1.0  # meters
```

## Troubleshooting

1. **Controller not detected**: Make sure your controller is connected before starting the Docker container. The Docker configuration has been updated to pass through controller devices.

2. **Line ending issues**: If you encounter errors related to line endings (e.g., `python3\r: No such file or directory`), the `run_joy_control.sh` script attempts to fix these automatically by using `dos2unix`.

3. **Joystick mapping issues**: Different controllers may have different button/axis mappings. You can use `jstest /dev/input/js0` in the container to test your controller and update the config file accordingly. 