from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare arguments
    autonomous_mode_arg = DeclareLaunchArgument(
        'autonomous_mode',
        default_value='true',
        description='Enable autonomous frontier exploration'
    )
    
    planning_time_arg = DeclareLaunchArgument(
        'planning_time', 
        default_value='2.0',
        description='Planning timeout in seconds'
    )

    return LaunchDescription([
        autonomous_mode_arg,
        planning_time_arg,
        
        # Autonomous Planner Node
        Node(
            package='rrtstar_octomap_planner',
            executable='autonomous_planner_node',
            name='autonomous_planner',
            output='screen',
            parameters=[{
                'autonomous_mode': LaunchConfiguration('autonomous_mode'),
                'planning_time': LaunchConfiguration('planning_time'),
                
                # Core parameters
                'vehicle_odometry_topic': '/fmu/out/vehicle_odometry',
                'octomap_topic': '/octomap_full', 
                'goal_topic': '/planner_goal',
                'planned_path_topic': '/planned_path',
                'frontier_topic': '/frontiers',
                'exploration_goal_topic': '/exploration_goal',
                
                # Safety parameters
                'collision_radius': 0.8,
                'safety_margin': 0.5,
                
                # BIT* parameters
                'samples_per_batch': 200,
                'max_attempts': 5,
                'max_waypoints': 25,
                
                # Autonomous exploration parameters
                'exploration_timeout': 30.0,
                'min_frontier_utility': 0.3,
                'frontier_update_interval': 2.0,
                
                # Workspace bounds
                'min_x': -50.0, 'max_x': 50.0,
                'min_y': -50.0, 'max_y': 50.0, 
                'min_z': 0.5, 'max_z': 10.0
            }]
        ),
        
        # Frontier Extraction Node
        Node(
            package='frontier_extraction',
            executable='frontier_3d_extraction_node',
            name='frontier_3d_extraction',
            output='screen',
            parameters=[{
                'min_frontier_points': 15,
                'tree_depth': 16,
                'max_point_distance_gain': 4.5,
                'max_dist_z_gain': 1.5,
                'only_ground_frontiers': False,
                'frontier_check.max_occupied_same_lv': 5,
                'frontier_check.max_occupied_up_lv': 3
            }]
        ),
        
        # Exploration Manager Node
        Node(
            package='exploration_manager', 
            executable='exploration_manager_node',
            name='exploration_manager',
            output='screen',
            parameters=[{
                'exploration_strategy': 'frontier',
                'update_frequency': 1.0,
                'goal_distance_threshold': 2.0,
                'frontier_min_size': 10,
                'max_exploration_radius': 25.0
            }]
        )
    ])