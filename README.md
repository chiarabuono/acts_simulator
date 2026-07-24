# ACTS Simulator (Aerial Cable-Towed System)

## Project Structure

Here is an overview of how the repository is organized:

```text
├── acts_simpler_cases/                  # Simplified simulation scripts
│   ├── mujoco_311_simulation.py
│   ├── mujoco_311_simulation_real.py   # Physical rig parameters
│   ├── mujoco_312_simulation.py
│   ├── mujoco_312_simulation_real.py   # Physical rig parameters
│   ├── mujoco_313_simulation.py
│   └── mujoco_single_drone.py
│
├── acts_simulator/                      # Core simulator module
│   ├── acts.py                         # Main simulation entry point
│   ├── params_acts.py                  # Geometric constraints, mass properties & bounds
│   ├── utils_configuration_selection.py# Architecture selector utilities
│   ├── utils_control.py                # Drone control laws
│   ├── utils_optimization.py           # Tension optimization & linear programming
│   ├── utils_performance_indices.py    # Performance indices & stability checks
│   ├── utils_visual.py                 # Plotting & visualization utilities
│   └── video_recorder.py               # Video capture tools
│
├── collected_data/                      # Output logs, metrics, and video renders
│   ├── indeces/                        # Plotted performance index & error graphs (.png)
│   ├── videos/                         # Recorded simulation runs (.mp4)
│   └── indices.xlsx                    # Tabulated performance metrics
│
├── create_xml/                          # Model generation & analytical screening toolkit
│   ├── analytical_ground_screening.py  # Fast screening algorithm for candidate configurations
│   ├── config_params.py                 # Configuration parameters for model creation
│   ├── poses_to_analyze.csv            # Evaluation pose dataset
│   ├── xml_config_builder.py           # Configuration builder
│   │
│   ├── database/                       # Configuration node & anchor databases
│   │   ├── uav_configuration_database.json
│   │   └── ugv_configuration_database.json
│   │
│   ├── images/                         # Reference architecture diagrams
│   │   ├── 3_uav_config.png
│   │   └── 6_ugv_config.jpeg
│   │
│   └── visual_interface/               # Interactive annotation & building UI
│       ├── tool_annotation_marker.py   # Interactive node coordinate annotation tool
│       ├── tool_crop_finder.py         # Interactive bounding box crop finder
│       └── xml_generator.py            # XML generator interface
│
├── mujoco/                              # MuJoCo XML configuration models
│   ├── acts_stewart.xml                # Stewart platform baseline model
│   ├── hand_made/                      # Manually crafted XML candidate models
│   ├── mujoco_outputs_1/               # Automated generator run output 1 (XMLs + screening CSV)
│   ├── mujoco_outputs_2/               # Automated generator run output 2 (XMLs + screening CSV)
│   ├── mujoco_outputs_3/               # Automated generator run output 3 (XMLs + screening CSV)
│   └── simpler_cases/                  # Isolated system XML components (311, 312, 313 models)
│
├── simulation_run/                      # Automation & batch testing
│   └── batch_run.py                    # Automated multi-configuration batch execution script
│
└── README.md
```

## Launch ACTS.py
If the system gives jacobian errors, try with:
```bash
pip uninstall -y cvxpy scipy numpy osqp ecos clarabel --break-system-packages
sudo apt install --reinstall python3-numpy python3-scipy
```

## Dependencies
- To create an xml file using the interactive guide pip install Pillow
- To plot pip install pyqtgraph PySide6