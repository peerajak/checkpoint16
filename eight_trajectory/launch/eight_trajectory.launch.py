import launch
from launch_ros.actions import Node

# How to use Example:
# ros2 launch execution_and_callbacks_examples start_with_arguments.launch.py timer_period:=0.5
#obstacle:=0.3 degrees:=-90 final_approach:=true

def generate_launch_description():
    return launch.LaunchDescription([
        # All the arguments have to be strings. Floats will give an error of NonItreable.
        Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}]         
        )
        ]
    )