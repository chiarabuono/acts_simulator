# Dependencies
To be able to use the TurtleBot3 package use: 
```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot3-description ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-msgs
sudo apt install ros-jazzy-ros-gz
sudo apt-get install libgz-sim8-plugins
```

Not sure that it is needed
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins' >> ~/.bashrc
source ~/.bashrc
```

# Commands to run it
```bash
colcon build --packages-select acts_simulator --symlink-install && source install/setup.bash && ros2 launch acts_simulator acts_simulation.launch.py
```
# Additional commands for debugging
To see graphically all the nodes

1. xacro acts.urdf.xacro > test.urdf
2. check_urdf test.urdf
3. urdf_to_graphiz test.urdf

All together: ``` xacro acts.urdf.xacro > test.urdf && check_urdf test.urdf && urdf_to_graphiz test.urdf ```

To see the state of the system
```bash
gz model -m acts_system -l
```

To send velocity to the motors
```bash
ros2 topic pub /drone1_/command/motor_speed actuator_msgs/msg/Actuators "{velocity: [0, 200, 0, 200]}" 
```

To see the state of joints
```bash
ros2 run tf2_ros tf2_echo drone1_base_link drone1_link_5 -r 1.0
```

To see the topic on ROS2 and Gazebo part
```bash
ros2 topic list
gz topic -l
```