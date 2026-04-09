# Dependencies
To be able to use the TurtleBot3 package use: 

sudo apt update
sudo apt install ros-jazzy-turtlebot3-description ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-msgs
sudo apt install ros-jazzy-ros-gz

## Commands to run it
colcon build --packages-select acts_simulator --symlink-install && source install/setup.bash && ros2 launch acts_simulator acts_simulation.launch.py

## To see graphically all the nodes
1 - xacro acts.urdf.xacro > test.urdf
2 - check_urdf test.urdf
3 - urdf_to_graphiz test.urdf

All together: xacro acts.urdf.xacro > test.urdf && check_urdf test.urdf && urdf_to_graphiz test.urdf

## To see the state of the system
gz model -m acts_system -l

ros2 topic pub /drone1_/command/motor_speed actuator_msgs/msg/Actuators "{velocity: [0, 200, 0, 200]}" 