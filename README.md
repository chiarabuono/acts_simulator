## Project Structure

Here is a quick overview of how the repository is organized:

```text
├── acts_simpler_cases/      # Simplified test scenarios
├── acts_simulator/          # Core simulator source package (physics, control, optimizations)
│   ├── acts.py              # Main simulation entry point
│   └── utils_control.py     # Drone control laws 
└── mujoco/                  # MuJoCo simulation model definitions
```