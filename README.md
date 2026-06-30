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
└──  mujoco/                      # MuJoCo simulation XML model definitions
    ├── acts_stewart.xml                # Complete integrated system model definition
    └── simpler_cases/                  # Individual XML definitions for the simpler configurations
```