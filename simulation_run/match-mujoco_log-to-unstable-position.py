"""
Matches MuJoCo "unstable DOF" warnings from a MUJOCO_LOG.TXT file to simulation records in an Excel log 
(by wall-clock time, refined by sim-time), and exports the matched rows to a new Excel file per folder.

Adapt: `target_folders` to simulation output directories, 
Check: `log_file_path`/`excel_filename`/`output_filename` match filenames,
Tune: `MAX_SIM_TIME_DELTA_S` to logging frequency.
"""

from datetime import datetime
import os
import re
import pandas as pd

# Adapt to simulation output directories
target_folders = [
    "./simulation_run/mujoco_outputs_ALIGNED",
    "./simulation_run/mujoco_outputs_DIAGONAL",
    "./simulation_run/mujoco_outputs_DISALIGNED",
    "./simulation_run/mujoco_outputs_LINE",
    "./simulation_run/mujoco_outputs_POINT",
    "./simulation_run/mujoco_outputs_TRIANGLE",
    ]
MAX_SIM_TIME_DELTA_S = 0.5 # Tune to logging frequency.

log_file_path = "MUJOCO_LOG.TXT"
excel_filename = "ground_cable_rubbing_log.xlsx"
output_filename = "parallel_matched_errors.xlsx"


mujoco_events = []
current_timestamp = None
current_time_str = None

if not os.path.exists(log_file_path):
    print(f"Error: Log file '{log_file_path}' not found.")
    exit(1)

with open(log_file_path, "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        ts_match = re.match(r"^[A-Z][a-z]{2}\s+[A-Z][a-z]{2}\s+\d+\s+(\d{2}:\d{2}:\d{2})\s+\d{4}", line,)
        if ts_match:
            current_time_str = ts_match.group(1)
            current_timestamp = datetime.strptime(current_time_str, "%H:%M:%S")
            continue

        if line.startswith("WARNING:") and current_timestamp:
            dof_match = re.search(r"DOF\s+(\d+)", line)
            sim_time_match = re.search(r"Time\s*=\s*(\d+(?:\.\d+)?)", line)

            dof = int(dof_match.group(1)) if dof_match else None
            sim_time = (float(sim_time_match.group(1)) if sim_time_match else None)

            mujoco_events.append({
                "wall_clock_time": current_time_str,
                "wall_clock_dt": current_timestamp,
                "warning_message": line,
                "unstable_dof": dof,
                "mujoco_sim_time": sim_time,
            })

df_errors = pd.DataFrame(mujoco_events)


for folder in target_folders:
    combined_excel_path = os.path.join(folder, excel_filename)
    output_excel_path = os.path.join(folder, output_filename)

    if not os.path.exists(combined_excel_path):
        print(f"Skipping {folder}: '{excel_filename}' not found.")
        continue

    df_excel = pd.read_excel(combined_excel_path)
    df_excel["wall_clock_dt"] = pd.to_datetime(
        df_excel["time"].astype(str), format="%H:%M:%S"
    )

    matched_rows = []

    for _, err in df_errors.iterrows():
        err_wall_time = err["wall_clock_dt"]
        err_sim_time = err["mujoco_sim_time"]

        time_window = df_excel[
            (df_excel["wall_clock_dt"] - err_wall_time).abs()
            <= pd.Timedelta(seconds=30)
        ].copy()

        if err_sim_time is not None and "simulation_time_s" in time_window.columns and not time_window.empty:
            time_window["sim_time_diff"] = (
                time_window["simulation_time_s"] - err_sim_time
            ).abs()

            best_match_idx = time_window["sim_time_diff"].idxmin()
            best_match = time_window.loc[best_match_idx]

            # ONLY append if the closest match falls within acceptable sim-time threshold
            if best_match["sim_time_diff"] <= MAX_SIM_TIME_DELTA_S:
                match_row = {
                    "error_wall_clock": err["wall_clock_time"],
                    "warning_message": err["warning_message"],
                    "unstable_dof": err["unstable_dof"],
                    "error_sim_time_s": err_sim_time,
                    "matched_model_xml": best_match.get("model_xml", "N/A"),
                    "matched_pose_index": best_match.get("pose_index", "N/A"),
                    "target_pos_x": best_match.get("target_pos_x", "N/A"),
                    "target_pos_y": best_match.get("target_pos_y", "N/A"),
                    "target_pos_z": best_match.get("target_pos_z", "N/A"),
                    "excel_sim_time_s": best_match.get("simulation_time_s", "N/A"),
                    "excel_wall_clock": best_match.get("time", "N/A"),
                    "sim_time_delta_s": round(best_match["sim_time_diff"], 4),
                }
                matched_rows.append(match_row)

    df_final = pd.DataFrame(matched_rows)

    with pd.ExcelWriter(output_excel_path, engine="openpyxl") as writer:
        df_final.to_excel(writer, sheet_name="Disambiguated_Errors", index=False)

    print(f"Processed {folder}: Found {len(df_final)} matching errors -> Saved to: {output_excel_path}")