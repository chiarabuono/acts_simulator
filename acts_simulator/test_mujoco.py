import mujoco
import mujoco.viewer
import time

# 1. Define a basic mechanical system using MuJoCo's XML format (MJCF)
# This creates a world with a floor grid, light, and a free-falling box.
xml_string = """
<mujoco>
    <option gravity="0 0 -9.81" timestep="0.002"/>
    
    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid_mat" texture="grid" texrepeat="10 10"/>
    </asset>

    <worldbody>
        <light pos="0 0 10" dir="0 0 -1"/>
        
        <geom type="plane" size="10 10 0.1" material="grid_mat"/>
        
        <body name="test_box" pos="0 0 4.0">
            <freejoint/> <geom type="box" size="0.2 0.2 0.2" rgba="1 0.5 0 1" mass="1.5"/>
        </body>
    </worldbody>
</mujoco>
"""

print("Loading MuJoCo model...")
# 2. Compile the string into a MuJoCo model and data state structure
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

print("Opening 3D Physics Viewer... (Close the graphics window to exit)")
# 3. Launch the native interactive 3D physics engine viewer loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        step_start = time.time()
        
        # Step the physics math forward by the model's timestep (0.002 seconds)
        mujoco.mj_step(model, data)
        
        # Periodically refresh the 3D graphics window
        viewer.sync()
        
        # Maintain close to real-time execution pacing
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("Simulation loop ended successfully.")