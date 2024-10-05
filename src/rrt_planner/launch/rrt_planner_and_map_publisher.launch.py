from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    rrt_planner_node = LifecycleNode(
        package='rrt_planner',
        executable='rrt_planner_node',
        name='rrt_planner_node',
        namespace='',
        output='screen',
        emulate_tty=True)
    
    rrt_star_planner_node = LifecycleNode(
        package='rrt_planner',
        executable='rrt_star_planner_node',
        name='rrt_star_planner_node',
        namespace='',
        output='screen',
        emulate_tty=True)

    rrt_connect_planner_node = LifecycleNode(
        package='rrt_planner',
        executable='rrt_connect_planner_node',
        name='rrt_connect_planner_node',
        namespace='',
        output='screen',
        emulate_tty=True)

    map_publisher_node = LifecycleNode(
        package='rrt_planner',
        executable='map_publisher',
        name='static_map_publisher',
        namespace='',
        output='screen',
        emulate_tty=True)

    # Build the launch description
    launch_description = LaunchDescription([
        rrt_planner_node,
        rrt_star_planner_node,
        rrt_connect_planner_node,
        map_publisher_node
    ])

    return launch_description