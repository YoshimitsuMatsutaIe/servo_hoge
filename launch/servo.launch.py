import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    
    
    controller = launch_ros.actions.Node(
        package='servo_hoge',
        node_executable='controller',
    )
    
    servo = launch_ros.actions.Node(
        package='servo_hoge',
        node_executable='servo',
    )
    
    return launch.LaunchDescription([controller, servo])