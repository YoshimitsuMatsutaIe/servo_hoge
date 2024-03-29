import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


# def generate_launch_description():
    
    
#     controller_2 = launch_ros.actions.Node(
#         package='servo_hoge',
#         node_executable='controller_2',
#         output='screen',
#     )
    
#     servo_2 = launch_ros.actions.Node(
#         package='servo_hoge',
#         node_executable='servo_2',
#         output='screen',
#     )
    
#     return launch.LaunchDescription([controller, servo_2,])


# def generate_launch_description():
#     """controllerとledのみ"""
    
#     controller = launch_ros.actions.Node(
#         package='servo_hoge',
#         node_executable='controller',
#         output='screen',
#     )
    
#     led = launch_ros.actions.Node(
#         package='servo_hoge',
#         node_executable='led',
#         output='screen',
#     )
    
#     return launch.LaunchDescription([controller, led])



def generate_launch_description():
    
    
    camera = launch_ros.actions.Node(
        package='servo_hoge',
        node_executable='camera',
        output='screen',
    )
    
    return launch.LaunchDescription([camera],)
