# ACTS Simulator (Aerial Cable-Towed System)

## Project Structure

Here is an overview of how the repository is organized:

```text
├── acts_simpler_cases/          # Simplified test scenarios
│   ├── mujoco_311_simulation.py        # 1 drone connected to the ground
│   ├── mujoco_311_simulation_real.py   # 311 simulation with physical rig parameters
│   ├── mujoco_312_simulation.py        # 1 drone, 1 payload, 1 ground cable simulation
│   ├── mujoco_312_simulation_real.py   # 312 simulation with physical rig parameters
│   ├── mujoco_313_simulation.py        # 1 drone, 1 payload, 2 ground cables simulation
│   └── mujoco_single_drone.py          # Single drone setup
│
├── acts_simulator/              # Core simulator
│   ├── acts.py                         # Main simulation entry point
│   ├── params_acts.py                  # Geometric constraints, mass properties, and bounds
│   ├── utils_control.py                # Drone control laws
│   ├── utils_optimization.py           # Tension planner
│   ├── utils_performance_indices.py    # Performance indices computation
│   └── utils_visual.py                 # Graphs and toolbar
│
├──  mujoco/                      # MuJoCo simulation XML model definitions
|   ├── acts_stewart.xml                # Complete integrated system model definition
|   └── simpler_cases/                  # Individual XML definitions for the simpler configurations
├──create_xml/
|    ├── assets/                          # Static reference images only
│    |   ├── 3_uav_config.png         
│    |   └── 6_ugv_config.jpeg        
│    |
|    ├── database/                        # Generated coordinates and metadata               
     │
     ├── config_params.py                 
     ├── tool_crop_finder.py              # Interactive tool to extract bounding box coordinates
     ├── tool_annotation_marker.py        # Interactive tool to annotate absolute 3D node coordinates
     └── xml_generator.py                 # Main file to create a customized
```

## Launch ACTS.py
If the system gives jacobian errors, try with:
```bash
pip uninstall -y cvxpy scipy numpy osqp ecos clarabel --break-system-packages
sudo apt install --reinstall python3-numpy python3-scipy
```

## Dependencies
- To create an xml file using the interactive guide pip install Pillow