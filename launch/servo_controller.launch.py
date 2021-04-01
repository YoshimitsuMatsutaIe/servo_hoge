import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    
    
    controller = launch_ros.actions.Node(
        package='servo_hoge',
        node_executable='controller',
        output='screen',
    )
    
    servo = launch_ros.actions.Node(
        package='servo_hoge',
        node_executable='servo',
        output='screen',
    )
    
    return launch.LaunchDescription([controller,])

# def generate_launch_description():
    
    
#     controller = launch_ros.actions.Node(
#         package='servo_hoge',
#         node_executable='controller',
#         output='screen',
#     )
    
#     return launch.LaunchDescription(
#         [controller]
#         )
