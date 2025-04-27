from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([

        GroupAction(
            actions=[
                Node(
                    package='lidar_to_image_depth',
                    executable='lidar_to_image_depth',
                    name='lidar_to_image_depth',
                    output='screen',
                    namespace='',
                    parameters=[PathJoinSubstitution([FindPackageShare('lidar_to_image_depth'), 'params', 'params.yaml'])]
                )
            ]
        )
    ])


