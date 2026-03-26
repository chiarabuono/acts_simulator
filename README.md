# Dependencies
To be able to use the TurtleBot3 package use: 

sudo apt update
sudo apt install ros-jazzy-turtlebot3-description ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-msgs
sudo apt install ros-jazzy-ros-gz

sudo apt install ros-jazzy-gz-ros2-control \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers

## Commands to run it
colcon build --packages-select acts_simulator --symlink-install && source install/setup.bash && ros2 launch acts_simulator acts_simulation.launch.py

## To see graphically all the nodes
1 - xacro acts.urdf.xacro > test.urdf
2 - check_urdf test.urdf
3 - urdf_to_graphiz test.urdf

All together: 
- ACTS system: xacro acts.urdf.xacro > test.urdf && check_urdf test.urdf && urdf_to_graphiz test.urdf
- Cable-pulley system: xacro adaptable_cable.xacro > test.urdf && check_urdf test.urdf && urdf_to_graphiz test.urdf

## To see the state of the system
gz model -m acts_system -l