from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    bond_timeout = LaunchConfiguration('bond_timeout')    
    bond_respawn_max_duration = LaunchConfiguration('bond_respawn_max_duration')
    attempt_respawn_reconnection = LaunchConfiguration('attempt_respawn_reconnection')

    rrt_planner_lifecycle_nodes = [
                           'rrt_planner_node',
                           'rrt_star_planner_node',
                           'rrt_connect_planner_node'
                                                   ]
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the RRT Planner')
    
    declare_bond_timeout_cmd = DeclareLaunchArgument(
        'bond_timeout', default_value='10.0',
        description='Bond timeout for RRT Planner')
    
    declare_bond_respawn_max_duration_cmd = DeclareLaunchArgument(
        'bond_respawn_max_duration', default_value='20.0',
        description='Bond respawn max duration for RRT Planner')
    
    declare_attempt_respawn_reconnection_cmd = DeclareLaunchArgument(
        'attempt_respawn_reconnection', default_value='true',
        description='Attempt respawn reconnection for RRT Planner')


    rrt_planner_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='rrt_planner_lifecycle_manager',
        output='screen',
        parameters=[{'autostart': autostart},
                    {'node_names': rrt_planner_lifecycle_nodes},
                    {'bond_timeout': bond_timeout},
                    {'bond_respawn_max_duration': bond_respawn_max_duration},
                    {'attempt_respawn_reconnection': attempt_respawn_reconnection}])


    return LaunchDescription(
        [
        declare_autostart_cmd,
        declare_bond_timeout_cmd,
        declare_bond_respawn_max_duration_cmd,
        declare_attempt_respawn_reconnection_cmd,
        rrt_planner_lifecycle_manager_node
        ]
    )
