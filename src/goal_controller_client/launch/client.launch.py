import launch
import launch_ros.actions


def generate_launch_description():
    client = launch_ros.actions.Node(
        package='goal_controller_client', executable='goal_controller_client', output='screen')
    return launch.LaunchDescription([
        client
    ])
