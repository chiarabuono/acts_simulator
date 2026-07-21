import tkinter as tk
import threading
from params_acts import ctrl_params
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

def run_tuning_gui():
    root = tk.Tk()
    root.title("Section 3.1.3 System Target & Gain Mixer")
    root.geometry("400x850")  # Slightly widened to account for gain scales neatly

    def update_val(key, val):
        ctrl_params[key] = float(val)

    # ==================== SECTION 1: TARGET POSITION ====================
    pos_frame = tk.LabelFrame(root, text="Desired Payload Position (p*)", font=('Helvetica', 10, 'bold'), fg="blue")
    pos_frame.pack(fill='x', padx=10, pady=5)

    tk.Label(pos_frame, text="Target X Position").pack(anchor='w', padx=5)
    s_px = tk.Scale(pos_frame, from_=-3.0, to=3.0, resolution=0.05, orient='horizontal', command=lambda v: update_val('px', v))
    s_px.set(ctrl_params['px'])
    s_px.pack(fill='x', padx=10, pady=2)

    tk.Label(pos_frame, text="Target Y Position").pack(anchor='w', padx=5)
    s_py = tk.Scale(pos_frame, from_=-2.0, to=2.0, resolution=0.05, orient='horizontal', command=lambda v: update_val('py', v))
    s_py.set(ctrl_params['py'])
    s_py.pack(fill='x', padx=10, pady=2)

    tk.Label(pos_frame, text="Target Z Position (Height)").pack(anchor='w', padx=5)
    s_pz = tk.Scale(pos_frame, from_=0.5, to=5.0, resolution=0.05, orient='horizontal', command=lambda v: update_val('pz', v))
    s_pz.set(ctrl_params['pz'])
    s_pz.pack(fill='x', padx=10, pady=2)


    # ==================== SECTION 2: TARGET ORIENTATION ====================
    quat_frame = tk.LabelFrame(root, text="Desired Payload Orientation (q*)", font=('Helvetica', 10, 'bold'), fg="purple")
    quat_frame.pack(fill='x', padx=10, pady=5)

    tk.Label(quat_frame, text="Quaternion W (Scalar)").pack(anchor='w', padx=5)
    s_qw = tk.Scale(quat_frame, from_=-1.0, to=1.0, resolution=0.01, orient='horizontal', command=lambda v: update_val('quat_w', v))
    s_qw.set(ctrl_params['quat_w'])
    s_qw.pack(fill='x', padx=10, pady=2)

    tk.Label(quat_frame, text="Quaternion X").pack(anchor='w', padx=5)
    s_qx = tk.Scale(quat_frame, from_=-1.0, to=1.0, resolution=0.01, orient='horizontal', command=lambda v: update_val('quat_x', v))
    s_qx.set(ctrl_params['quat_x'])
    s_qx.pack(fill='x', padx=10, pady=2)

    tk.Label(quat_frame, text="Quaternion Y").pack(anchor='w', padx=5)
    s_qy = tk.Scale(quat_frame, from_=-1.0, to=1.0, resolution=0.01, orient='horizontal', command=lambda v: update_val('quat_y', v))
    s_qy.set(ctrl_params['quat_y'])
    s_qy.pack(fill='x', padx=10, pady=2)

    tk.Label(quat_frame, text="Quaternion Z").pack(anchor='w', padx=5)
    s_qz = tk.Scale(quat_frame, from_=-1.0, to=1.0, resolution=0.01, orient='horizontal', command=lambda v: update_val('quat_z', v))
    s_qz.set(ctrl_params['quat_z'])
    s_qz.pack(fill='x', padx=10, pady=2)


    # ==================== SECTION 3: CONTROLLER GAINS ====================
    # gain_frame = tk.LabelFrame(root, text="Controller PID Tuning Gains", font=('Helvetica', 10, 'bold'), fg="darkgreen")
    # gain_frame.pack(fill='x', padx=10, pady=5)

    # tk.Label(gain_frame, text="Kp Position Gain").pack(anchor='w', padx=5)
    # s_kp = tk.Scale(gain_frame, from_=0.0, to=100.0, resolution=0.5, orient='horizontal', command=lambda v: update_val('Kp_pos', v))
    # s_kp.set(ctrl_params['Kp_pos'])
    # s_kp.pack(fill='x', padx=10, pady=2)

    # tk.Label(gain_frame, text="Kd Position Derivative Gain").pack(anchor='w', padx=5)
    # s_kd = tk.Scale(gain_frame, from_=0.0, to=50.0, resolution=0.1, orient='horizontal', command=lambda v: update_val('Kd_pos', v))
    # s_kd.set(ctrl_params['Kd_pos'])
    # s_kd.pack(fill='x', padx=10, pady=2)

    # tk.Label(gain_frame, text="Kr Attitude Orientation Gain").pack(anchor='w', padx=5)
    # s_kr = tk.Scale(gain_frame, from_=0.0, to=50.0, resolution=0.5, orient='horizontal', command=lambda v: update_val('Kr', v))
    # s_kr.set(ctrl_params['Kr'])
    # s_kr.pack(fill='x', padx=10, pady=2)

    # tk.Label(gain_frame, text="Kw Angular Velocity Damping Gain").pack(anchor='w', padx=5)
    # s_kw = tk.Scale(gain_frame, from_=0.0, to=20.0, resolution=0.1, orient='horizontal', command=lambda v: update_val('Kw', v))
    # s_kw.set(ctrl_params['Kw'])
    # s_kw.pack(fill='x', padx=10, pady=2)

    root.mainloop()

gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()

 
class LiveIndexPlot:
 
    GROUPS = {
        "indeces": ["conditioning_index", "worst_case_capacity_margin"],        #TODO: give best name
        "wrench_margin": ["capacity_margin", "radius_available_wrench"],
        "manipulability": ["manipulability"],
        "composite_score" : ["composite_score"]
    }
 
    def __init__(self, max_points: int = 500):
        plt.ion()
        self.fig, axes = plt.subplots(4, 1, sharex=True, figsize=(7, 8))
        self.axes = {
            "indeces": axes[0],
            "wrench_margin": axes[1],
            "manipulability": axes[2],
            "composite_score" : axes[3]
        }
 
        self.t = deque(maxlen=max_points)
        self.series = {}
        self.lines = {}
 
        for group_name, keys in self.GROUPS.items():
            ax = self.axes[group_name]
            for k in keys:
                self.series[k] = deque(maxlen=max_points)
                self.lines[k] = ax.plot([], [], label=k)[0]
            ax.legend(loc="upper right", fontsize=8)
 
        self.axes["wrench_margin"].axhline(0, color="gray", linestyle="--", linewidth=0.8)
        # self.axes["normalized"].set_ylabel("index, choose a better name")
        # self.axes["wrench_margin"].set_ylabel("wrench margin")
        # self.axes["manipulability"].set_ylabel("manipulability")
        self.axes["composite_score"].set_xlabel("time [s]")
 
        self.fig.tight_layout()
 
    def update(self, t: float, indices: dict):
        self.t.append(t)
 
        for k, line in self.lines.items():
            value = indices.get(k, np.nan)
            self.series[k].append(value)
            line.set_data(self.t, self.series[k])
 
        for ax in self.axes.values():
            ax.relim()
            ax.autoscale_view()
 
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
 
    def close(self):
        plt.close(self.fig)


plot = LiveIndexPlot()
