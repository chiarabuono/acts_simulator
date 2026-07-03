import os
import json
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk

# Clean absolute import from our data parameters module
from config_params import (
    IMAGE_PATH_UGV, IMAGE_PATH_UAV, UGV_DB_PATH, UAV_DB_PATH,
    GRID_MAPPING_UGV, GRID_MAPPING_UAV, MUJOCO_TEMPLATE
)

class MultiStepSelectorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MuJoCo Configuration Wizard")
        self.root.geometry("950x950")

        # Load databases safely
        self.ugv_geo_db = self._load_json_db(UGV_DB_PATH)
        self.uav_geo_db = self._load_json_db(UAV_DB_PATH)
        if not self.ugv_geo_db or not self.uav_geo_db:
            return

        # Setup steps wizard array
        self.stages = [
            {"title": "Select Payload Attachment Layout (UGV)", "image_path": IMAGE_PATH_UGV, "options": list(GRID_MAPPING_UGV.keys()), "key": "ugv_config_payload", "mapping": GRID_MAPPING_UGV},
            {"title": "Select Ground Anchor Layout (UGV)", "image_path": IMAGE_PATH_UGV, "options": list(GRID_MAPPING_UGV.keys()), "key": "ugv_config_ground", "mapping": GRID_MAPPING_UGV},
            {"title": "Select UAV Flight Setup Configuration", "image_path": IMAGE_PATH_UAV, "options": list(GRID_MAPPING_UAV.keys()), "key": "uav_config", "mapping": GRID_MAPPING_UAV}
        ]
        
        self.current_stage_idx = 0
        self.selections = {}
        self.setup_ui_containers()
        self.load_stage(self.current_stage_idx)

    def _load_json_db(self, path):
        if os.path.exists(path):
            with open(path, "r") as f:
                return json.load(f)
        messagebox.showerror("Missing File", f"Could not find '{path}'. Run annotations first!")
        self.root.destroy()
        return None

    def setup_ui_containers(self):
        self.left_frame = ttk.Frame(self.root, padding=10, width=320)
        self.left_frame.pack(side=tk.LEFT, fill=tk.Y, expand=False)
        self.left_frame.pack_propagate(False)

        self.right_frame = ttk.Frame(self.root, padding=10)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.lbl_title = ttk.Label(self.left_frame, text="", font=("Arial", 12, "bold"), wraplength=300)
        self.lbl_title.pack(anchor=tk.W, pady=(0, 10))

        list_container = ttk.Frame(self.left_frame)
        list_container.pack(fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(list_container, orient=tk.VERTICAL)
        self.listbox = tk.Listbox(list_container, yscrollcommand=scrollbar.set, font=("Arial", 11))
        scrollbar.config(command=self.listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.btn_action = ttk.Button(self.left_frame, text="Next Step →", command=self.handle_next)
        self.btn_action.pack(fill=tk.X, pady=(10, 0))

        self.lbl_img = ttk.Label(self.right_frame)
        self.lbl_img.pack(fill=tk.BOTH, expand=True, anchor=tk.CENTER)

    def load_stage(self, stage_idx):
        stage_data = self.stages[stage_idx]
        self.lbl_title.config(text=stage_data["title"])
        self.listbox.delete(0, tk.END)
        for option in stage_data["options"]:
            self.listbox.insert(tk.END, option)
        
        if os.path.exists(stage_data["image_path"]):
            img = Image.open(stage_data["image_path"])
            # Targeted layout scaling
            size_limit = (600, 600) if stage_data["key"] == "uav_config" else (900, 900)
            img.thumbnail(size_limit)
            
            self.photo = ImageTk.PhotoImage(img)
            self.lbl_img.config(image=self.photo)

    def handle_next(self):
        selection = self.listbox.curselection()
        if not selection:
            messagebox.showwarning("Selection Required", "Please select an option before moving forward.")
            return

        current_stage = self.stages[self.current_stage_idx]
        self.selections[current_stage["key"]] = self.listbox.get(selection[0])

        if self.current_stage_idx < len(self.stages) - 1:
            self.current_stage_idx += 1
            self.load_stage(self.current_stage_idx)
        else:
            self.clear_window_for_parameters()

    def clear_window_for_parameters(self):
        for widget in self.root.winfo_children():
            widget.destroy()
        self.load_parameter_interface()

    def load_parameter_interface(self):
        self.root.title("System Cable Routing Configuration Grid")
        
        left_control_panel = ttk.Frame(self.root, padding=15, width=450)
        left_control_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        left_control_panel.pack_propagate(False)

        right_image_panel = ttk.Frame(self.root, padding=15, width=650)
        right_image_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        right_image_panel.pack_propagate(False)

        pay_layout = self.selections.get('ugv_config_payload')
        gnd_layout = self.selections.get('ugv_config_ground')
        uav_layout = self.selections.get('uav_config')

        # Stacked canvases
        ttk.Label(right_image_panel, text=f"Payload Target (UGV): {pay_layout.upper()}", font=("Arial", 9, "bold")).pack(anchor=tk.W)
        self.canvas_pay = tk.Canvas(right_image_panel, height=220, bg="white", relief="groove", bd=1)
        self.canvas_pay.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(right_image_panel, text=f"Ground Base Target (UGV): {gnd_layout.upper()}", font=("Arial", 9, "bold")).pack(anchor=tk.W)
        self.canvas_gnd = tk.Canvas(right_image_panel, height=220, bg="white", relief="groove", bd=1)
        self.canvas_gnd.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(right_image_panel, text=f"UAV Rigid Frame Shape: {uav_layout.upper()}", font=("Arial", 9, "bold")).pack(anchor=tk.W)
        self.canvas_uav = tk.Canvas(right_image_panel, height=220, bg="white", relief="groove", bd=1)
        self.canvas_uav.pack(fill=tk.X)

        self.load_and_overlay_reference(IMAGE_PATH_UGV, self.canvas_pay, pay_layout, GRID_MAPPING_UGV, self.ugv_geo_db)
        self.load_and_overlay_reference(IMAGE_PATH_UGV, self.canvas_gnd, gnd_layout, GRID_MAPPING_UGV, self.ugv_geo_db)
        self.load_and_overlay_reference(IMAGE_PATH_UAV, self.canvas_uav, uav_layout, GRID_MAPPING_UAV, self.uav_geo_db)

        # Dropdowns table setup
        ttk.Label(left_control_panel, text="Cable Routing Settings", font=("Arial", 14, "bold")).pack(anchor=tk.W, pady=(0, 15))
        pay_letters = list(self.ugv_geo_db.get(pay_layout, {}).keys())
        gnd_letters = list(self.ugv_geo_db.get(gnd_layout, {}).keys())

        headers_frame = ttk.Frame(left_control_panel)
        headers_frame.pack(fill=tk.X, pady=5)
        ttk.Label(headers_frame, text="Cable ID", font=("Arial", 10, "bold"), width=12).pack(side=tk.LEFT)
        ttk.Label(headers_frame, text="Payload Node", font=("Arial", 10, "bold"), width=18).pack(side=tk.LEFT)
        ttk.Label(headers_frame, text="Ground Node", font=("Arial", 10, "bold"), width=18).pack(side=tk.LEFT)

        self.routing_variables = {}
        for cable_num in range(4, 10):
            row_frame = ttk.Frame(left_control_panel, padding=2)
            row_frame.pack(fill=tk.X, pady=2)
            ttk.Label(row_frame, text=f"Cable {cable_num}:", font=("Arial", 11), width=12).pack(side=tk.LEFT)

            pay_var = tk.StringVar()
            combo_pay = ttk.Combobox(row_frame, textvariable=pay_var, values=pay_letters, width=10, state="readonly")
            combo_pay.pack(side=tk.LEFT, padx=(0, 35))
            if pay_letters: combo_pay.current(0)

            gnd_var = tk.StringVar()
            combo_gnd = ttk.Combobox(row_frame, textvariable=gnd_var, values=gnd_letters, width=10, state="readonly")
            combo_gnd.pack(side=tk.LEFT)
            if gnd_letters: combo_gnd.current(0)

            self.routing_variables[f"cable_{cable_num}"] = {"payload_anchor": pay_var, "ground_anchor": gnd_var}

        btn_generate = ttk.Button(left_control_panel, text="Compile MuJoCo XML File", command=self.process_final_xml_data)
        btn_generate.pack(fill=tk.X, side=tk.BOTTOM, pady=15)

    def load_and_overlay_reference(self, path, target_canvas, config_name, mapping, db):
        if os.path.exists(path) and config_name in mapping:
            orig_img = Image.open(path)
            crop_box = mapping[config_name]
            cropped_snippet = orig_img.crop(crop_box)
            
            crop_w, crop_h = cropped_snippet.size
            cropped_snippet.thumbnail((450, 200))
            thumb_w, thumb_h = cropped_snippet.size
            
            scale_x, scale_y = thumb_w / crop_w, thumb_h / crop_h
            
            photo_ref = ImageTk.PhotoImage(cropped_snippet)
            setattr(target_canvas, f"photo_{config_name}", photo_ref)
            target_canvas.create_image(10, 10, image=photo_ref, anchor=tk.NW)

            config_nodes = db.get(config_name, {})
            for node_letter, data in config_nodes.items():
                pixel_loc = data.get("visual_annotation_pixel", [0, 0])
                cx = int(pixel_loc[0] * scale_x) + 10
                cy = int(pixel_loc[1] * scale_y) + 10
                
                if pixel_loc[0] != 0 or pixel_loc[1] != 0:
                    target_canvas.create_oval(cx-10, cy-10, cx+10, cy+10, fill="yellow", outline="orange", width=2)
                    target_canvas.create_text(cx, cy, text=node_letter, font=("Arial", 9, "bold"), fill="black")

    def process_final_xml_data(self):
        pay_layout = self.selections.get('ugv_config_payload')
        gnd_layout = self.selections.get('ugv_config_ground')
        uav_layout = self.selections.get('uav_config')
        
        # --- VALIDATE DISCRETE CAPACITY & DUPLICATE PATH CONSTRAINTS ---
        payload_connection_counts = {}
        ground_connection_counts = {}
        seen_routing_pairs = set()
        validation_errors = []

        for cable_id, vars_dict in self.routing_variables.items():
            num = cable_id.split("_")[1]
            p_node = vars_dict["payload_anchor"].get()
            g_node = vars_dict["ground_anchor"].get()
            
            if (p_node, g_node) in seen_routing_pairs:
                validation_errors.append(f"• Cable {num} attempts duplicate path: '{p_node}' ↔ '{g_node}'")
            else:
                seen_routing_pairs.add((p_node, g_node))

            payload_connection_counts[p_node] = payload_connection_counts.get(p_node, 0) + 1
            ground_connection_counts[g_node] = ground_connection_counts.get(g_node, 0) + 1

        for nodes_dict, db_source, label in [(payload_connection_counts, self.ugv_geo_db[pay_layout], "Payload"), (ground_connection_counts, self.ugv_geo_db[gnd_layout], "Ground")]:
            for letter, count in nodes_dict.items():
                max_allowed = db_source[letter].get("max_cables", 999)
                if count > max_allowed:
                    validation_errors.append(f"• {label} Node '{letter}': Connected {count} (Max: {max_allowed})")

        if validation_errors:
            messagebox.showerror("Routing Errors Detected", "Fix errors:\n" + "\n".join(validation_errors))
            return

        # --- CONSTRUCT STRINGS ---
        payload_sites, ground_sites, tendon_elements, actuator_elements, sensor_elements = "", "", "", "", ""
        for cable_id, vars_dict in self.routing_variables.items():
            num = cable_id.split("_")[1]
            p_node, g_node = vars_dict["payload_anchor"].get(), vars_dict["ground_anchor"].get()
            px, py, pz = self.ugv_geo_db[pay_layout][p_node]["coords"]
            gx, gy, gz = self.ugv_geo_db[gnd_layout][g_node]["coords"]
            
            payload_sites += f'            <site name="hook_{num}" pos="{px} {py} {phz if "phz" in locals() else pz}" size="0.06" rgba="1 0.2 0.2 1"/>\n'
            ground_sites += f'        <site name="ground_anchor_{num}" pos="{gx} {gy} {gz}" size="0.07" rgba="1 0.2 0.2 1"/>\n'
            tendon_elements += f'        <spatial name="cable_{num}" limited="true" range="0 40.0" width="0.015" rgba="1 0.3 0.3 1">\n            <site site="ground_anchor_{num}"/>\n            <site site="hook_{num}"/>\n        </spatial>\n'
            actuator_elements += f'      <position name="cable_{num}_winch" tendon="cable_{num}" kp="2000" kv="150" ctrlrange="0 40.0"/>\n'
            sensor_elements += f'    <tendonlimitfrc name="cable_{num}_tension" tendon="cable_{num}"/>\n'

        uav_bodies_string, uav_tendons_string, uav_actuators_string, uav_sensors_string = "", "", "", ""
        uav_nodes = self.uav_geo_db.get(uav_layout, {})
        for idx, node_key in enumerate(sorted(list(uav_nodes.keys()))[:3]):
            drone_idx = idx + 1
            ux, uy, uz = uav_nodes[node_key]["coords"]
            phx, phy, phz = self.ugv_geo_db[pay_layout].get(chr(65 + idx), {}).get("coords", [0.0, 0.0, 0.1])
            
            payload_sites += f'            <site name="hook_{drone_idx}" pos="{phx} {phy} {phz}" size="0.04" rgba="1 1 0 1"/>\n'
            uav_bodies_string += f'        <body name="drone_{drone_idx}" pos="{ux} {uy} 0.20">\n            <freejoint name="drone_{drone_idx}_joint"/>\n            <inertial pos="0 0 0" mass="2.0" diaginertia="0.01 0.01 0.015"/>\n            <geom name="drone_{drone_idx}_geom" type="cylinder" size="0.15 0.05" rgba="0 0.7 0.9 1" mass="2.0" condim="3" friction="1 0.005 0.0001"/>\n            <site name="drone_{drone_idx}_com" pos="0 0 0" size="0.02" rgba="1 1 0 1"/>\n        </body>\n'
            uav_tendons_string += f'        <spatial name="cable_{drone_idx}" limited="true" range="0 1.5" width="0.015" rgba="0 0.8 0 1">\n            <site site="drone_{drone_idx}_com"/>\n            <site site="hook_{drone_idx}"/>\n        </spatial>\n'
            uav_actuators_string += f'      <motor name="drone_{drone_idx}_thrust" site="drone_{drone_idx}_com" gear="0 0 1 0 0 0"/>\n      <motor name="drone_{drone_idx}_roll"   site="drone_{drone_idx}_com" gear="0 0 0 1 0 0"/>\n      <motor name="drone_{drone_idx}_pitch"  site="drone_{drone_idx}_com" gear="0 0 0 0 1 0"/>\n      <motor name="drone_{drone_idx}_yaw"    site="drone_{drone_idx}_com" gear="0 0 0 0 0 1"/>\n'
            uav_sensors_string += f'    <tendonlimitfrc name="cable_{drone_idx}_tension" tendon="cable_{drone_idx}"/>\n'

        xml_content = MUJOCO_TEMPLATE.format(
            payload_sites=payload_sites, uav_bodies_string=uav_bodies_string, ground_sites=ground_sites,
            uav_tendons_string=uav_tendons_string, tendon_elements=tendon_elements, uav_actuators_string=uav_actuators_string,
            actuator_elements=actuator_elements, uav_sensors_string=uav_sensors_string, sensor_elements=sensor_elements
        )

        # --- SAVE ARCHITECTURE LOOKUP ---
        os.makedirs("mujoco", exist_ok=True)
        base_name = f"mujoco/{pay_layout.upper()}-{gnd_layout.upper()}-{uav_layout.upper()}"
        
        current_fingerprint = sorted([(vars_dict["payload_anchor"].get(), vars_dict["ground_anchor"].get()) for vars_dict in self.routing_variables.values()])
        output_filename = f"{base_name}.xml"
        counter = 2
        message_detail = ""

        while os.path.exists(output_filename):
            existing_fingerprint = []
            try:
                with open(output_filename, "r") as ef:
                    for line in ef:
                        if '<spatial name="cable_' in line:
                            s1, s2 = next(ef, ""), next(ef, "")
                            if "ground_anchor_" in s1 and "hook_" in s2:
                                g = s1.split('ground_anchor_')[1].split('"')[0]
                                p = s2.split('hook_')[1].split('"')[0]
                                if not g.isdigit() and not p.isdigit(): existing_fingerprint.append((p, g))
            except (IOError, StopIteration):
                existing_fingerprint = None

            if existing_fingerprint is not None and sorted(existing_fingerprint) == current_fingerprint:
                message_detail = f"Identical network layout already exists at:\n'{output_filename}'"
                break
            output_filename = f"{base_name}-{counter}.xml"
            counter += 1

        if not os.path.exists(output_filename) and message_detail == "":
            with open(output_filename, "w") as f: f.write(xml_content)
            message_detail = f"XML saved to:\n'{output_filename}'"

        messagebox.showinfo("Compilation Status", message_detail)

if __name__ == "__main__":
    root = tk.Tk()
    app = MultiStepSelectorApp(root)
    root.mainloop()