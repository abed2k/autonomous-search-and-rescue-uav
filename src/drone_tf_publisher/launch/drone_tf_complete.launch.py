#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        use_sim_time_arg,
        
        # Dynamic transform from PX4 odometry (map -> base_link)
        Node(
            package='drone_tf_publisher',  # Your package name
            executable='px4_pose_tf',      # Your existing node executable
            name='odom_to_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('fmu/out/vehicle_odometry', 'fmu/out/vehicle_odometry'),
            ]
        ),
        
        # Static transform (base_link -> lidar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_base_to_lidar',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0', 
                '--z', '0.25',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'x500_0/lidar_3d_link/lidar_3d'
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
    ])