import launch
from launch_ros.actions import Node



def generate_launch_description():
    return launch.LaunchDescription([
        
        Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}]         
        )
        ]
    )