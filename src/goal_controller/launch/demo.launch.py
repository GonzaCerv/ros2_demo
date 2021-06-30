import launch
import launch_ros.actions


def generate_launch_description():
    ros_params = {"distance_threshold": 0.5,
                "linear_speed": 1.5,
                "max_linear_speed": 4.0,
                "angular_speed": 3.0,
                "max_angular_speed": 4.0}
    turtlesim = launch_ros.actions.Node(
        package='turtlesim', executable='turtlesim_node', output='screen')
    server = launch_ros.actions.Node(
        package='goal_controller', executable='goal_controller_action_server', output='screen', parameters=[ros_params])
    client = launch_ros.actions.Node(
        package='goal_controller_client', executable='goal_controller_client', output='screen')
    
    return launch.LaunchDescription([
        turtlesim,
        server,
        client,
    ])
