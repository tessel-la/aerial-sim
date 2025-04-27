import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # bag_file = "/root/aerostack2_ws/src/lidar_to_image_depth/calib_bag_11_43/calib_bag_11_43_0.db3"
    bag_file = "/root/aerostack2_ws/src/lidar_to_image_depth/calib_bag_12_11/calib_bag_12_11_0.db3"

    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', bag_file, '--loop'],
        #     output='screen'
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_livox_to_camera',
            # arguments=[
            #     '-0.29802171074891903',
            #     '0.09941277017855925',
            #     '-0.142296103676214',
            #     '0.3520613584692897',
            #     '0.4008434392204727',
            #     '0.5803728134248316',
            #     '0.6152598918701001',
            #     'livox_frame',
            #     'pylon_camera'
            # ],

            arguments=[
                # '-0.142296103676214',
                # '0.09941277017855925',
                # '0.29802171074891903',
                '-0.29802171074891903',
                '0.09941277017855925',
                '-0.2296103676214',
                '0.3520613584692897',
                '0.4008434392204727',
                '0.5803728134248316',
                '0.6152598918701001',
                'livox_frame',
                'camera_base_link'
            ],

            # arguments=[
            #     '-0.142296103676214',
            #     '0.09941277017855925',
            #     '0.29802171074891903',
            #     '0.3742',  # qx
            #     '0.3742',  # qy
            #     '0.6009',  # qz
            #     '0.6009',  # qw
            #     'livox_frame',
            #     'pylon_camera'
            # ],

        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=[
                '0.3', '0', '-0.2', "0.9842639", "0", "0.1749802", "-0.0246269",
                'base_link',
                'livox_frame'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=[
                '0.0', '0.0', '0.0', "0.0", "0.0", "0.0", "1",
                'gimbal_link',
                'pylon_camera'
            ],
        ),
    ])