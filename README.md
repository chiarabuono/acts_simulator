# Dependencies
To be able to use the TurtleBot3 package use: 

sudo apt update
sudo apt install ros-jazzy-turtlebot3-description ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-msgs
sudo apt install ros-jazzy-ros-gz

## Other dependancies that have been installed 
- From gz_attach_links
```bash
sudo apt install ros-${ROS_DISTRO}-simple-launch ros-${ROS_DISTRO}-slider-publisher
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-pip libeigen3-dev
```
- From ls2n_drone_disturbance_observer
```bash
git clone https://github.com/ICube-Robotics/acados_vendor_ros2.git
cd acados_vendor_ros2
git checkout v0.3.6
rosdep install --from-paths . -y --ignore-src
sudo apt install ros-$ROS_DISTRO-ament-cmake-vendor-package
cd your_ros2_ws
colcon build --packages-select acados_vendor_ros2

git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_disturbance_observer.git
sudo apt update && sudo apt install python3-venv
cd ls2n_drone_disturbance_observer
python3 -m venv .venv
source .venv/bin/activate
pip3 install -r python-requirements.txt
cd your_ros2_ws
colcon build --packages-select ls2n_drone_disturbance_observer
deactivate
```

- From ls2n_drone_trajectories
```bash
git clone git@gitlab.univ-nantes.fr:ls2n-drones/ls2n_drone_interfaces.git
git clone git@gitlab.univ-nantes.fr:ls2n-drones/ls2n_drone_description.git
git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_trajectories.git
git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_joystick.git

cd ros2_ws/src
git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_interfaces.git
git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_joystick.git

colcon build --packages-select ls2n_drone_interfaces ls2n_drone_description gz_attach_links ls2n_drone_trajectories ls2n_drone_joystick
source install/local_setup.bash
```

- From ls2n_drone_interfaces (maybe not)

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ls2n_drone_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release
source $ROS2_WS/install/setup.bash
``

- From ls2n_drone_px4_autopilot

1. PX4-Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl
bash Tools/setup/ubuntu.sh
make px4_sitl
git clone git@gitlab.univ-nantes.fr:ls2n-drones/ls2n_drone_px4_autopilot.git
```
(idk if make px4_sitl should go before or after)
        
2.  The actual package
```bash
sudo apt-get install python3-venv
sudo apt install python3-typeguard python3-jinja2
colcon build --cmake-args -DPX4_VERSION=v1.15.4 --cmake-args -DINSTALL_PX4_DEPS=ON --packages-select ls2n_drone_px4_autopilot

```
- From ls2n_drone_command_centre
```bash
git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_command_center.git
git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_bridge.git
git clone git@gitlab.univ-nantes.fr:ls2n-drones/ls2n_drone_qualisys.git
source ~/your_ros2_ws/src/ls2n_drone_qualisys/.venv/bin/activate
pip install qtm-rt
python3 -m pip install numpy transforms3d
deactivate
cd ls2n_drone_qualisys
git submodule update --init --recursive
colcon build --packages-select ls2n_drone_command_center ls2n_drone_bridge ls2n_drone_qualisys --symlink-install
source install/setup.bash
```

- From
```bash
git clone https://gitlab.univ-nantes.fr/ls2n-drones/ls2n_drone_controllers.git
colcon build --packages-select ls2n_drone_controllers --symlink-install
source install/setup.bash
```

RUN armada with
Once the packages are built, source your directory and install the trajectories:
```bash
source /path_to_your_ros2_ws
ros2 run ls2n_drone_trajectories trajectory_generator.py
ros2 run ls2n_drone_trajectories trajectory_generator.py ls2n_drone_armada
colcon build --packages-select ls2n_drone_armada --symlink-install
source install/setup.bash
chmod +x ~/your_ros2_ws/src/ls2n_drone_armada-main/ls2n_drone_armada/odometry_transformer.py
```
Finally:
```bash
ros2 launch ls2n_drone_armada sitl_armada_control.launch.py
ros2 launch ls2n_drone_armada sitl_armada_control.launch.py drones_to_control:="['crazy2fly1', 'crazy2fly2', 'crazy2fly3', 'crazy2fly4']" drone_masses:="[1.2, 1.2, 1.2, 1.2]" platform_mass:=1.0 use_cpp:=True gz_world:=armada_world_net armada_controller:=model-free
```

In the doubt: source install/setup.bash

## Commands to run it
colcon build --packages-select acts_simulator --symlink-install && source install/setup.bash && ros2 launch acts_simulator acts_simulation.launch.py

## To see graphically all the nodes
1 - xacro acts.urdf.xacro > test.urdf
2 - check_urdf test.urdf
3 - urdf_to_graphiz test.urdf

All together: xacro acts.urdf.xacro > test.urdf && check_urdf test.urdf && urdf_to_graphiz test.urdf

## To see the state of the system
gz model -m acts_system -l

## To create videos of the simulation
sudo apt install ros-jazzy-ros-gzsim-vendors libgz-sim8-plugins
sudo apt install ros-jazzy-ros-gz-sim libgz-sim8-gui-plugins-all
sudo apt install ros-jazzy-image-view

sudo apt update && sudo apt install wl-screenrec
sudo apt update && sudo apt install gnome-monitor-recorder


ros2 topic pub /drone1_/command/motor_speed actuator_msgs/msg/Actuators "{velocity: [0, 200, 0, 200]}" 