import launch
from launch_ros.actions import Node



def generate_launch_description():
    return launch.LaunchDescription([
        
        Node(
            package='wheel_velocities_publisher',
            executable='wheel_velocities_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}]         
        ),
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}]         
        )
        ]
    )