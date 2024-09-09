# checkpoint16

## Task 1
Terminal 1.
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py

Terminal 2.
source ~/ros2_ws/install/setup.bash
ros2 launch kinematic_model kinematic_model.launch.py

## Task 2
Terminal 1.
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py

Terminal 2.
source ~/ros2_ws/install/setup.bash
ros2 launch eight_trajectory eight_trajectory.launch.py

## Helper
Try to pub /cmdvel at 10Hz for 1 second
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.5708" --rate 10 -t 10

