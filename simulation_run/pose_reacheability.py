"""
Checks a batch-run results.xlsx and reports which target poses (grouped by position+quaternion) were never reached by any model configuration.
Adapt: set `xlsx` to the results file to analyze.
"""

import pandas as pd

xlsx = 'simulation_run/mujoco_outputs_TRIANGLE/results.xlsx' # Change with the file to analyze
df = pd.read_excel(xlsx)

df.columns = df.columns.str.strip()
pose_cols = ['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z']
df['pose_reached'] = df['pose_reached'].fillna(False).astype(bool)
pose_reachability = df.groupby(pose_cols)['pose_reached'].any().reset_index()
unreached_poses = pose_reachability[pose_reachability['pose_reached'] == False]

print(f"Total Unique Poses Evaluated: {len(pose_reachability)}")
print(f"Unreached Poses Count:        {len(unreached_poses)}")

if not unreached_poses.empty:
    print(f"File analyzed: {xlsx}")
    print("\n[!] The following poses were NOT reached by ANY configuration:\n")
    print(unreached_poses[pose_cols].to_string(index=False))
else:
    print("\n[✓] All poses were reached by at least one configuration!")