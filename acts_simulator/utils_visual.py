import sys
import tkinter as tk
import threading
from acts_simulator.params_acts import ctrl_params
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R
import pyqtgraph as pg
import pyqtgraph.exporters
from pyqtgraph.Qt import QtCore, QtWidgets

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
pg.setConfigOptions(useOpenGL=True, antialias=True)

# ==================== TKINTER TUNING GUI THREAD ====================
def run_tuning_gui():
    root = tk.Tk()
    root.title("Section 3.1.3 System Target & Gain Mixer")
    root.geometry("400x850")

    def update_val(key, val):
        ctrl_params[key] = float(val)

    # Positional Controls
    pos_frame = tk.LabelFrame(root, text="Desired Payload Position (p*)", fg="blue")
    pos_frame.pack(fill='x', padx=10, pady=5)
    for k in ['px', 'py', 'pz']:
        tk.Label(pos_frame, text=f"Target {k.upper()}").pack(anchor='w', padx=5)
        s = tk.Scale(pos_frame, from_=-3.0 if k!='pz' else 0.5, to=3.0 if k!='pz' else 8.0, 
                     resolution=0.05, orient='horizontal', command=lambda v, key=k: update_val(key, v))
        s.set(ctrl_params[k])
        s.pack(fill='x', padx=10, pady=2)

    # Quaternion Controls
    quat_frame = tk.LabelFrame(root, text="Desired Payload Orientation (q*)", fg="purple")
    quat_frame.pack(fill='x', padx=10, pady=5)
    for k in ['quat_w', 'quat_x', 'quat_y', 'quat_z']:
        tk.Label(quat_frame, text=f"Quaternion {k.split('_')[1].upper()}").pack(anchor='w', padx=5)
        s = tk.Scale(quat_frame, from_=-1.0, to=1.0, resolution=0.01, orient='horizontal', 
                     command=lambda v, key=k: update_val(key, v))
        s.set(ctrl_params[k])
        s.pack(fill='x', padx=10, pady=2)

    root.mainloop()

gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()


# ==================== INDEX PLOT DASHBOARD ====================
class LiveIndexPlot(QtWidgets.QMainWindow):
    def __init__(self, max_points: int = 500):
        super().__init__()
        self.setWindowTitle("Performance Indices Telemetry")
        self.resize(700, 800)

        self.max_points = max_points
        self.t = deque(maxlen=max_points)
        self.series = {}
        self.lines = {}

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        self.win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.win)

        self.ax_indeces = self.win.addPlot(title="Indeces")
        self.win.nextRow()
        self.ax_wrench = self.win.addPlot(title="Wrench Margin")
        self.win.nextRow()
        self.ax_manip = self.win.addPlot(title="Manipulability")
        self.win.nextRow()
        self.ax_composite = self.win.addPlot(title="Composite Score")

        for ax in [self.ax_indeces, self.ax_wrench, self.ax_manip, self.ax_composite]:
            ax.showGrid(x=False, y=False, alpha=0.3)
            legend = ax.addLegend(offset=(10, 10), labelTextSize='12pt')
            legend.setLabelTextSize('12pt')

        zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#7f7f7f', style=QtCore.Qt.DashLine))
        self.ax_wrench.addItem(zero_line)

        # High-contrast pens for white background
        self._add_line(self.ax_indeces, "conditioning_index", '#d62728')
        self._add_line(self.ax_indeces, "worst_case_capacity_margin", '#2ca02c')
        self._add_line(self.ax_wrench, "capacity_margin", '#1f77b4')
        self._add_line(self.ax_wrench, "radius_available_wrench", '#17becf')
        self._add_line(self.ax_manip, "manipulability", '#9467bd')
        self._add_line(self.ax_composite, "composite_score", '#e377c2')

    def _add_line(self, ax, key: str, color_hex: str):
        self.series[key] = deque(maxlen=self.max_points)
        self.lines[key] = ax.plot(pen=pg.mkPen(color=color_hex, width=3), name=key)

    def update(self, t: float, indices: dict):
        self.t.append(t)
        for k in self.lines.keys():
            self.series[k].append(indices.get(k, np.nan))

        t_arr = np.array(self.t)
        for key, line in self.lines.items():
            line.setData(t_arr, np.array(self.series[key]))

    def export_image(self, filepath: str = "index_plot.png"):
        """Saves high-res snapshot of the dashboard without needing to right-click."""
        filepath = f"collected_data/indeces/{filepath}"
        exporter = pg.exporters.ImageExporter(self.win.ci)  # Targets central layout item
        exporter.export(filepath)
        print(f"Exported Index Plot to {filepath}")


# ==================== ERROR PLOT DASHBOARD ====================
class LiveErrorPlot(QtWidgets.QMainWindow):
    def __init__(self, max_points: int = 500):
        super().__init__()
        self.setWindowTitle("System Tracking Errors")
        self.resize(700, 500)

        self.max_points = max_points
        self.t = deque(maxlen=max_points)
        self.series = {}
        self.lines = {}

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        self.win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.win)

        self.ax_pos = self.win.addPlot(title="Position Error [m]")
        self.win.nextRow()
        self.ax_rot = self.win.addPlot(title="Attitude Error [rad]")

        for ax in [self.ax_pos, self.ax_rot]:
            ax.showGrid(x=False, y=False, alpha=0.3)
            legend = ax.addLegend(offset=(10, 10), labelTextSize='12pt')
            legend.setLabelTextSize('12pt')

        self._add_line(self.ax_pos, "x", '#d62728')
        self._add_line(self.ax_pos, "y", '#2ca02c')
        self._add_line(self.ax_pos, "z", '#1f77b4')

        self._add_line(self.ax_rot, "roll", '#17becf')
        self._add_line(self.ax_rot, "pitch", '#9467bd')
        self._add_line(self.ax_rot, "yaw", '#e377c2')

    def _add_line(self, ax, key: str, color_hex: str):
        self.series[key] = deque(maxlen=self.max_points)
        self.lines[key] = ax.plot(pen=pg.mkPen(color=color_hex, width=3), name=f"e_{key}")

    def update(self, t: float, p_payload: np.ndarray, R_mat_payload: np.ndarray, target_pos: np.ndarray, target_quat: np.ndarray):
        self.t.append(t)

        e_pos = target_pos - p_payload
        self.series["x"].append(e_pos[0])
        self.series["y"].append(e_pos[1])
        self.series["z"].append(e_pos[2])

        q_norm = target_quat / (np.linalg.norm(target_quat) + 1e-9)
        R_d = R.from_quat([q_norm[1], q_norm[2], q_norm[3], q_norm[0]]).as_matrix()
        
        R_err = R_mat_payload.T @ R_d
        e_rot = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1]
        ])
        self.series["roll"].append(e_rot[0])
        self.series["pitch"].append(e_rot[1])
        self.series["yaw"].append(e_rot[2])

        t_arr = np.array(self.t)
        for key, line in self.lines.items():
            line.setData(t_arr, np.array(self.series[key]))

    def export_image(self, filepath: str = "error_plot.png"):
        """Saves high-res snapshot of the dashboard without needing to right-click."""
        filepath = f"collected_data/indeces/{filepath}"
        exporter = pg.exporters.ImageExporter(self.win.ci)  # Targets central layout item
        exporter.export(filepath)
        print(f"Exported Error Plot to {filepath}")