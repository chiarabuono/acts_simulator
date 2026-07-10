import os
import json
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Clean absolute import from our data parameters module
from config_params import (
    IMAGE_PATH_UGV, IMAGE_PATH_UAV, UGV_DB_PATH, UAV_DB_PATH,
    GRID_MAPPING_UGV, GRID_MAPPING_UAV, MUJOCO_TEMPLATE,
    _load_json_db
)

class MultiStepSelectorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MuJoCo Configuration Wizard")
        self.root.geometry("950x950")

        # Load databases safely
        self.ugv_geo_db = _load_json_db(UGV_DB_PATH)
        self.uav_geo_db = _load_json_db(UAV_DB_PATH)
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
        pay_letters = [k for k in self.ugv_geo_db.get(pay_layout, {}).keys() if k != "symmetry_metadata"]
        gnd_letters = [k for k in self.ugv_geo_db.get(gnd_layout, {}).keys() if k != "symmetry_metadata"]

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
        
        
        ttk.Separator(left_control_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)
        ttk.Label(left_control_panel, text="Ground Layout Scaling", font=("Arial", 11, "bold")).pack(anchor=tk.W, pady=(0, 5))

        # Size mode dropdown setup
        self.size_mode_var = tk.StringVar(value="Normal")
        combo_size = ttk.Combobox(left_control_panel, textvariable=self.size_mode_var, values=["Small", "Normal", "Large"], state="readonly")
        combo_size.pack(fill=tk.X, pady=2)

        # Dynamic multiplier input frame
        self.scale_factor_frame = ttk.Frame(left_control_panel)
        ttk.Label(self.scale_factor_frame, text="How many times smaller/larger?", font=("Arial", 10)).pack(anchor=tk.W, pady=(5, 2))
        self.scale_factor_var = tk.StringVar(value="2.0")  # Default multiplier factor
        entry_scale = ttk.Entry(self.scale_factor_frame, textvariable=self.scale_factor_var)
        entry_scale.pack(fill=tk.X)

        # Function to toggle the entry visibility based on the dropdown choice
        def toggle_scale_entry(*args):
            if self.size_mode_var.get() in ["Small", "Large"]:
                self.scale_factor_frame.pack(fill=tk.X, before=combo_size.pack_info().get("before")) # packs it nicely below dropdown
                self.scale_factor_frame.pack(fill=tk.X, pady=(5, 0))
            else:
                self.scale_factor_frame.pack_forget()

        self.size_mode_var.trace_add("write", toggle_scale_entry)
        
        
        ttk.Separator(left_control_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)
        ttk.Label(left_control_panel, text="Symmetry Mirroring (UAV Only)", font=("Arial", 11, "bold")).pack(anchor=tk.W, pady=(0, 5))

        # Core tracking boolean states for UAV
        self.mirror_x_var = tk.BooleanVar(value=False)
        self.mirror_y_var = tk.BooleanVar(value=False)

        # Build the dynamic checkbox widgets for UAV
        self.chk_uav_x = ttk.Checkbutton(left_control_panel, text="Mirror Drone Positions along X-axis", variable=self.mirror_x_var)
        self.chk_uav_y = ttk.Checkbutton(left_control_panel, text="Mirror Drone Positions along Y-axis", variable=self.mirror_y_var)

        # Read the UAV database metadata for the specific shape chosen
        uav_layout = self.selections.get('uav_config')
        uav_layout_data = self.uav_geo_db.get(uav_layout, {})
        uav_metadata = uav_layout_data.get("symmetry_metadata", {"is_x_symmetric": False, "is_y_symmetric": False})

        # Dynamically pack UAV checkboxes based on its geometry layout structural data
        if not uav_metadata.get("is_x_symmetric", False):
            self.chk_uav_x.pack(anchor=tk.W, pady=2)
        else:
            self.chk_uav_x.pack_forget()

        if not uav_metadata.get("is_y_symmetric", False):
            self.chk_uav_y.pack(anchor=tk.W, pady=2)
        else:
            self.chk_uav_y.pack_forget()

        # --- PLACE THE NEW CODE HERE (Just before the compile button) ---
        ttk.Separator(left_control_panel, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)
        ttk.Label(left_control_panel, text="Ground Base Mirroring Options", font=("Arial", 11, "bold")).pack(anchor=tk.W, pady=(0, 5))

        # Core tracking boolean states
        self.mirror_gnd_x_var = tk.BooleanVar(value=False)
        self.mirror_gnd_y_var = tk.BooleanVar(value=False)

        # Build the checkbox widgets
        self.chk_gnd_x = ttk.Checkbutton(left_control_panel, text="Mirror Ground Layout along X-axis", variable=self.mirror_gnd_x_var)
        self.chk_gnd_y = ttk.Checkbutton(left_control_panel, text="Mirror Ground Layout along Y-axis", variable=self.mirror_gnd_y_var)

        # Read the database metadata for the specific layout chosen by the user in Step 2
        gnd_layout = self.selections.get('ugv_config_ground')
        layout_data = self.ugv_geo_db.get(gnd_layout, {})
        metadata = layout_data.get("symmetry_metadata", {"is_x_symmetric": False, "is_y_symmetric": False})

        # ONLY show the checkbox if the layout is NOT already symmetric on that axis
        if not metadata.get("is_x_symmetric", False):
            self.chk_gnd_x.pack(anchor=tk.W, pady=2)
        else:
            self.chk_gnd_x.pack_forget()

        if not metadata.get("is_y_symmetric", False):
            self.chk_gnd_y.pack(anchor=tk.W, pady=2)
        else:
            self.chk_gnd_y.pack_forget()

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
            
            target_canvas.create_image(225, 110, image=photo_ref, anchor=tk.CENTER)

            config_nodes = db.get(config_name, {})
            for node_letter, data in config_nodes.items():
                pixel_loc = data.get("visual_annotation_pixel", [0, 0])
                
                cx = int(pixel_loc[0] * scale_x) + (225 - thumb_w // 2)
                cy = int(pixel_loc[1] * scale_y) + (110 - thumb_h // 2)
                
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
                if letter == "symmetry_metadata":
                    continue
                max_allowed = db_source[letter].get("max_cables", 999)
                if count > max_allowed:
                    validation_errors.append(f"• {label} Node '{letter}': Connected {count} (Max: {max_allowed})")

        if validation_errors:
            messagebox.showerror("Routing Errors Detected", "Fix errors:\n" + "\n".join(validation_errors))
            return

        # --- VALIDATE AND CALCULATE GROUND SCALE FACTOR ---
        scale_mode = self.size_mode_var.get()
        factor = 1.0

        if scale_mode in ["Small", "Large"]:
            try:
                factor = float(self.scale_factor_var.get())
                if factor <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Invalid Input", "Please enter a valid positive number for the scaling factor.")
                return


        # --- CONSTRUCT STRINGS (CABLES 4-9: GROUND WINCHES TO LOWER FACADE) ---
        payload_sites, ground_sites, tendon_elements, actuator_elements, sensor_elements = "", "", "", "", ""
        
        gnd_mx = -1.0 if (self.chk_gnd_x.winfo_manager() and self.mirror_gnd_x_var.get()) else 1.0
        gnd_my = -1.0 if (self.chk_gnd_y.winfo_manager() and self.mirror_gnd_y_var.get()) else 1.0

        for cable_id, vars_dict in self.routing_variables.items():
            num = cable_id.split("_")[1]
            p_node = vars_dict["payload_anchor"].get()
            g_node = vars_dict["ground_anchor"].get()
            
            px, py, _ = self.ugv_geo_db[pay_layout][p_node]["coords"]  # Ignore raw pz
            raw_gx, raw_gy, _ = self.ugv_geo_db[gnd_layout][g_node]["coords"]

            if scale_mode == "Small":
                gx = raw_gx / factor
                gy = raw_gy / factor
            elif scale_mode == "Large":
                gx = raw_gx * factor
                gy = raw_gy * factor
            else:
                gx = raw_gx
                gy = raw_gy
            
            gx = gx * gnd_mx
            gy = gy * gnd_my
            gz = 0.0

            # Force the hook Z coordinate to the LOWER facade
            payload_sites += f'            <site name="hook_{num}" pos="{px} {py} -0.10" size="0.06" rgba="1 0.2 0.2 1"/>\n'
            ground_sites += f'        <site name="ground_anchor_{num}" pos="{gx} {gy} {gz}" size="0.07" rgba="1 0.2 0.2 1"/>\n'
            tendon_elements += f'        <spatial name="cable_{num}" limited="true" range="0 40.0" width="0.015" rgba="1 0.3 0.3 1">\n            <site site="ground_anchor_{num}"/>\n            <site site="hook_{num}"/>\n        </spatial>\n'
            actuator_elements += f'      <position name="cable_{num}_winch" tendon="cable_{num}" kp="2000" kv="150" ctrlrange="0 40.0"/>\n'
            sensor_elements += f'    <tendonlimitfrc name="cable_{num}_tension" tendon="cable_{num}"/>\n'

        # --- DYNAMIC UAV STRUCT (EXACT JSON X/Y FOR HOOKS) ---
        uav_bodies_string, uav_tendons_string, uav_actuators_string, uav_sensors_string = "", "", "", ""


        cables = {}
        for key in self.uav_geo_db[uav_layout]:
            if key == "symmetry_metadata": continue
            cables[key] = self.uav_geo_db[uav_layout][key]["max_cables"]

        drone_idx = 0
        for letter in cables:
            raw_px, raw_py, _ = self.uav_geo_db[uav_layout][letter]["coords"]

            while cables[letter] != 0:
                if cables[letter] == 1:
                    ux = raw_px + 0.20
                    uy = raw_py + 0.20
                if cables[letter] == 2:
                    ux = raw_px - 0.20
                    uy = raw_py - 0.20
                if cables[letter] == 3:
                    ux = raw_px + 0.20
                    uy = raw_py - 0.20
                uz = 0.60
                drone_idx += 1
                
                payload_sites += f'       <site name="hook_{drone_idx}" pos="{raw_px} {raw_py} 0.10" size="0.04" rgba="1 1 0 1"/>\n'
                cables[letter] -= 1

                uav_bodies_string += f'        <body name="drone_{drone_idx}" pos="{ux:.3f} {uy:.3f} {uz:.3f}">\n            <freejoint name="drone_{drone_idx}_joint"/>\n            <inertial pos="0 0 0" mass="2.0" diaginertia="0.01 0.01 0.015"/>\n            <geom name="drone_{drone_idx}_geom" type="cylinder" size="0.15 0.05" rgba="0 0.7 0.9 1" mass="2.0" condim="3" friction="1 0.005 0.0001"/>\n            <site name="drone_{drone_idx}_com" pos="0 0 0" size="0.02" rgba="1 1 0 1"/>\n        </body>\n'
                uav_tendons_string += f'        <spatial name="cable_{drone_idx}" limited="true" range="0 1.5" width="0.015" rgba="0 0.8 0 1">\n            <site site="drone_{drone_idx}_com"/>\n            <site site="hook_{drone_idx}"/>\n        </spatial>\n'
                uav_actuators_string += f'      <motor name="drone_{drone_idx}_thrust" site="drone_{drone_idx}_com" gear="0 0 1 0 0 0"/>\n      <motor name="drone_{drone_idx}_roll"   site="drone_{drone_idx}_com" gear="0 0 0 1 0 0"/>\n      <motor name="drone_{drone_idx}_pitch"  site="drone_{drone_idx}_com" gear="0 0 0 0 1 0"/>\n      <motor name="drone_{drone_idx}_yaw"    site="drone_{drone_idx}_com" gear="0 0 0 0 0 1"/>\n'
                uav_sensors_string += f'    <tendonlimitfrc name="cable_{drone_idx}_tension" tendon="cable_{drone_idx}"/>\n'

        xml_content = MUJOCO_TEMPLATE.format(
            payload_sites=payload_sites, uav_bodies_string=uav_bodies_string, ground_sites=ground_sites,
            uav_tendons_string=uav_tendons_string, tendon_elements=tendon_elements, uav_actuators_string=uav_actuators_string,
            actuator_elements=actuator_elements, uav_sensors_string=uav_sensors_string, sensor_elements=sensor_elements
        )

        # --- SAVE ARCHITECTURE LOOKUP (COORDINATE FINGERPRINT) ---
        import re

        os.makedirs("mujoco/hand_made", exist_ok=True)
        base_name = f"mujoco/hand_made/{pay_layout.upper()}-{gnd_layout.upper()}-{uav_layout.upper()}"
        
        if scale_mode == "Small":
            base_name += f"-small-{factor}".replace('.', '_')
        elif scale_mode == "Large":
            base_name += f"-large-{factor}".replace('.', '_')

        if self.mirror_x_var.get() and self.mirror_y_var.get():
            base_name += "-mirrorred-xy"
        elif self.mirror_x_var.get():
            base_name += "-mirrorred-x"
        elif self.mirror_y_var.get():
            base_name += "-mirrorred-y"

        current_coords_fingerprint = []
        for vars_dict in self.routing_variables.values():
            p_node = vars_dict["payload_anchor"].get()
            g_node = vars_dict["ground_anchor"].get()
            
            px, py, _ = self.ugv_geo_db[pay_layout][p_node]["coords"]
            gx, gy, gz = self.ugv_geo_db[gnd_layout][g_node]["coords"]
            
            p_str = f"{px} {py} -0.10"  # Fingerprint matches lower facade
            g_str = f"{gx} {gy} {gz}"
            current_coords_fingerprint.append((p_str, g_str))
            
        current_coords_fingerprint.sort()

        output_filename = f"{base_name}.xml"
        counter = 2
        message_detail = ""

        while os.path.exists(output_filename):
            existing_coords_fingerprint = []
            try:
                with open(output_filename, "r") as ef:
                    file_content = ef.read()
                
                for cable_num in range(4, 10):
                    p_pos_match = re.search(r'<site name="hook_' + str(cable_num) + r'" pos="([^"]+)"', file_content)
                    g_pos_match = re.search(r'<site name="ground_anchor_' + str(cable_num) + r'" pos="([^"]+)"', file_content)
                    
                    if p_pos_match and g_pos_match:
                        existing_coords_fingerprint.append((p_pos_match.group(1).strip(), g_pos_match.group(1).strip()))
                        
            except IOError:
                existing_coords_fingerprint = None

            if existing_coords_fingerprint:
                existing_coords_fingerprint.sort()

            if existing_coords_fingerprint == current_coords_fingerprint:
                message_detail = f"An identical network topology already exists at:\n'{output_filename}'\n\nNo duplicate file was written."
                break
            
            output_filename = f"{base_name}-{counter}.xml"
            counter += 1

        if not os.path.exists(output_filename) and message_detail == "":
            with open(output_filename, "w") as f: 
                f.write(xml_content)
            message_detail = f"XML saved to:\n'{output_filename}'"

        messagebox.showinfo("Compilation Status", message_detail)

if __name__ == "__main__":
    root = tk.Tk()
    app = MultiStepSelectorApp(root)
    root.mainloop()