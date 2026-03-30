# Dependencies
Before everything: ```sudo apt update ```. Then

- TurtleBot3 package 
``` 
ros-jazzy-turtlebot3-description ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-msgs
```
- Pulleys
```
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-ros2-control \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-effort-controllers
```
- Graph (run it with ```ros2 run plotjuggler plotjuggler```)
```
sudo apt install ros-jazzy-plotjuggler-ros
```


# Run the package
``` colcon build --packages-select acts_simulator --symlink-install && source install/setup.bash && ros2 launch acts_simulator acts_simulation.launch.py ```

# Debug
You can see all the Gazebo links and joints with the following commands (in the URDF folder):
1. ```xacro acts.urdf.xacro > test.urdf```
2. ```check_urdf test.urdf```
3. ```urdf_to_graphiz test.urdf```

All together: 
```
ACTS system: xacro acts.urdf.xacro > test.urdf && check_urdf test.urdf && urdf_to_graphiz test.urdf

Cable-pulley system: xacro adaptable_cable.xacro > test.urdf && check_urdf test.urdf && urdf_to_graphiz test.urdf
```
You can see the state of the system with 
```gz model -m acts_system -l```

# How to use the position controlled (cable release) pulley:
- Create a physical pulley in "cable_guide.urdf.xacro" to ensure that the gazebo cable goes through the right attachment point
- Add in "adaptable_cable.xacro" the pulley to ensure the gazebo connection
- Give the right parameters to the controller in cable_controller.launch.py:

```
parameters=[{
    # Cable extreme position
            "current_x": float(init_x),             
            "current_y": float(init_y),
            "current_z": float(init_z),
    
    # Cable length
            "total_cable_len": float(cable_len),

    # At which extreme of the cable the pulley is connected (link_first or link_last)
            "cable_extreme": cable_extreme,
    
    # Pulley velocity
            "vel": 0.01,
    
    # Amount of cable released at the beginning and the desired one at the end
            "unwinded_cable_len": float(unwinded_len),
            "final_cable_len": float(final_unwinded_cable_len),

    # Pulley position
            "pulley_x": float(pulley_x),
            "pulley_y": float(pulley_y),
            "pulley_z": float(pulley_z),            
        }]
```

#TODO: Something can be optimized, if I have the pulley position and how much cable is released, I don't need to say where the extreme is