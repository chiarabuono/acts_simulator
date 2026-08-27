"""
Interactive Tkinter tool for building a node-layout database: 
for each named layout template (line, triangle, rhombus, etc., cropped from a reference image per MODE), 
lets you enter node count, local (x,y,z) coordinates and max-cable-capacity per node, 
drag labels onto the reference image to visually confirm placement, auto-detects X/Y symmetry, 
and exports everything to a JSON database.

Adapt: 
    MODE ("uav" or "ugv")
    GRID_MAPPING from the tool `tool_crop_finder.py` if the base configurations change
"""
import os
import json
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

MODE = "uav" # or ugv

class AnnotatedDatabaseBuilder:
    def __init__(self, mode, root, img_path, GRID_MAPPING, json_path):
        self.mode = mode
        self.root = root
        self.root.title("Robotic Topology & Visual Layout Mapping Tool")
        self.root.geometry("1100x750")
        self.JSON_PATH = json_path

        self.img_path = img_path
        self.options_to_map = list(GRID_MAPPING.keys())
        self.current_idx = 0
        self.database = {}

        # Tracking for draggable canvas objects
        self.draggable_labels = {}
        self.selected_label_id = None

        self.setup_ui()
        self.load_active_option()

    def setup_ui(self):
        # Master Panels layout split
        self.left_panel = ttk.Frame(self.root, padding=10, width=500)
        self.left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.left_panel.pack_propagate(False)

        self.right_panel = ttk.Frame(self.root, padding=10)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # --- LEFT PANEL ELEMENTS ---
        self.lbl_title = ttk.Label(self.left_panel, text="", font=("Arial", 14, "bold"), foreground="darkblue")
        self.lbl_title.pack(pady=10, anchor=tk.W)

        step1_frame = ttk.Frame(self.left_panel, padding=5)
        step1_frame.pack(fill=tk.X)

        ttk.Label(step1_frame, text="How many points/nodes?").pack(side=tk.LEFT, padx=5)
        self.ent_points = ttk.Entry(step1_frame, width=8)
        self.ent_points.pack(side=tk.LEFT, padx=5)
        
        btn_generate = ttk.Button(step1_frame, text="Configure & Label Nodes", command=self.build_node_rows)
        btn_generate.pack(side=tk.LEFT, padx=5)

        # Scroll area for tabular values entry
        scroll_container = ttk.Frame(self.left_panel, padding=5)
        scroll_container.pack(fill=tk.BOTH, expand=True, pady=10)

        self.table_canvas = tk.Canvas(scroll_container)
        scrollbar = ttk.Scrollbar(scroll_container, orient="vertical", command=self.table_canvas.yview)
        self.scrollable_frame = ttk.Frame(self.table_canvas)

        self.scrollable_frame.bind("<Configure>", lambda e: self.table_canvas.configure(scrollregion=self.table_canvas.bbox("all")))
        self.table_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.table_canvas.configure(yscrollcommand=scrollbar.set)
        
        self.table_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Bottom Next Action Button
        btn_save = ttk.Button(self.left_panel, text="Save Layout Matrix & Next →", command=self.save_current_layout)
        btn_save.pack(fill=tk.X, side=tk.BOTTOM, pady=10)

        # --- RIGHT PANEL ELEMENTS (Visual Canvas) ---
        ttk.Label(self.right_panel, text="Visual Reference (Drag letters to match drawing nodes)", font=("Arial", 11, "bold")).pack(anchor=tk.W, pady=(0, 5))
        
        self.image_canvas = tk.Canvas(self.right_panel, bg="white", borderwidth=2, relief="groove")
        self.image_canvas.pack(fill=tk.BOTH, expand=True)

        # Canvas Mouse events bindings for Drag-and-Drop Labeling
        self.image_canvas.bind("<ButtonPress-1>", self.on_label_click)
        self.image_canvas.bind("<B1-Motion>", self.on_label_drag)

        self.node_inputs = []

    def load_active_option(self):
        """Switches the actively selected geometry target configuration template."""
        if self.current_idx < len(self.options_to_map):
            target = self.options_to_map[self.current_idx]
            self.lbl_title.config(text=f"Mapping Matrix: {target.upper()} ({self.current_idx + 1}/{len(self.options_to_map)})")
            
            self.ent_points.delete(0, tk.END)
            for widget in self.scrollable_frame.winfo_children():
                widget.destroy()
            self.node_inputs = []
            
            self.image_canvas.delete("all")
            self.draggable_labels.clear()

            if os.path.exists(self.img_path):
                orig_img = Image.open(self.img_path)
                crop_box = GRID_MAPPING[target]
                cropped_snippet = orig_img.crop(crop_box)
                cropped_snippet.thumbnail((500, 500))
                
                self.photo = ImageTk.PhotoImage(cropped_snippet)
                self.image_canvas.create_image(10, 10, image=self.photo, anchor=tk.NW, tags="bg_image")
        else:
            self.export_database_to_file(self.JSON_PATH)

    def build_node_rows(self):
        """Builds numeric parameter table rows and spawns draggable layout markers."""
        for widget in self.scrollable_frame.winfo_children():
            widget.destroy()
        
        for lbl_id in self.draggable_labels.keys():
            self.image_canvas.delete(lbl_id)
        self.draggable_labels.clear()
        self.node_inputs = []

        try:
            num_points = int(self.ent_points.get())
        except ValueError:
            messagebox.showerror("Error", "Enter a valid integer for points count.")
            return

        headers = ["Node", "Local X", "Local Y", "Local Z", "Max Cables"]
        for col_idx, text in enumerate(headers):
            ttk.Label(self.scrollable_frame, text=text, font=("Arial", 9, "bold")).grid(row=0, column=col_idx, padx=5, pady=5)

        for i in range(num_points):
            node_letter = chr(65 + i)
            row = i + 1

            ttk.Label(self.scrollable_frame, text=f"Point {node_letter}:").grid(row=row, column=0, padx=5, pady=2)

            if self.mode == "uav": z = "0.1"
            elif self.mode == "ugv": z = "-0.1"
            else: print("Error in the mode selected")
            
            ent_x = ttk.Entry(self.scrollable_frame, width=7); ent_x.insert(0, "0.0"); ent_x.grid(row=row, column=1, padx=2)
            ent_y = ttk.Entry(self.scrollable_frame, width=7); ent_y.insert(0, "0.0"); ent_y.grid(row=row, column=2, padx=2)
            ent_z = ttk.Entry(self.scrollable_frame, width=7); ent_z.insert(0, z); ent_z.grid(row=row, column=3, padx=2)
            
            cable_cap = ttk.Combobox(self.scrollable_frame, values=["1", "2", "3", "4"], width=4, state="readonly")
            cable_cap.current(0); cable_cap.grid(row=row, column=4, padx=5)

            self.node_inputs.append({"letter": node_letter, "x": ent_x, "y": ent_y, "z": ent_z, "capacity": cable_cap})

            spawn_x = 30 + (i * 35)
            spawn_y = 35
            
            circle_id = self.image_canvas.create_oval(spawn_x-12, spawn_y-12, spawn_x+12, spawn_y+12, fill="yellow", outline="orange", width=2, tags="marker")
            text_id = self.image_canvas.create_text(spawn_x, spawn_y, text=node_letter, font=("Arial", 11, "bold"), fill="black", tags="marker")
            
            self.draggable_labels[text_id] = {"circle": circle_id, "letter": node_letter}
            self.draggable_labels[circle_id] = {"text": text_id, "letter": node_letter}

    def on_label_click(self, event):
        clicked_item = self.image_canvas.find_closest(event.x, event.y)
        if clicked_item and clicked_item[0] in self.draggable_labels:
            self.selected_label_id = clicked_item[0]

    def on_label_drag(self, event):
        if self.selected_label_id is not None:
            tgt = self.selected_label_id
            paired_dict = self.draggable_labels[tgt]
            
            if "circle" in paired_dict:
                text_id, circle_id = tgt, paired_dict["circle"]
            else:
                text_id, circle_id = paired_dict["text"], tgt

            self.image_canvas.coords(text_id, event.x, event.y)
            self.image_canvas.coords(circle_id, event.x-12, event.y-12, event.x+12, event.y+12)

    def save_current_layout(self):
        """Compiles configurations, evaluates spatial symmetries automatically, and builds entry structures."""
        if not self.node_inputs:
            messagebox.showwarning("Warning", "Configure and place nodes first.")
            return

        current_target = self.options_to_map[self.current_idx]
        nodes_dictionary = {}

        visual_positions = {}
        for item_id, details in self.draggable_labels.items():
            if "circle" in details:
                coords = self.image_canvas.coords(item_id)
                visual_positions[details["letter"]] = [int(coords[0]), int(coords[1])]

        try:
            coords_list = []
            for item in self.node_inputs:
                let = item["letter"]
                cx = float(item["x"].get())
                cy = float(item["y"].get())
                cz = float(item["z"].get())
                coords_list.append([cx, cy, cz])

                nodes_dictionary[let] = {
                    "coords": [cx, cy, cz],
                    "max_cables": int(item["capacity"].get()),
                    "visual_annotation_pixel": visual_positions.get(let, [0, 0])
                }

            # --- AUTOMATED SYMMETRY EVALUATION MATRIX ---
            is_x_symmetric = True
            is_y_symmetric = True
            tolerance = 1e-4

            for pt in coords_list:
                x, y, _ = pt
                has_x_partner = any(abs(other[0] + x) < tolerance and abs(other[1] - y) < tolerance for other in coords_list)
                has_y_partner = any(abs(other[0] - x) < tolerance and abs(other[1] + y) < tolerance for other in coords_list)
                
                if not has_x_partner:
                    is_x_symmetric = False
                if not has_y_partner:
                    is_y_symmetric = False

            # Wrap inside targeted master dataset structure
            final_layout_block = {
                "symmetry_metadata": {
                    "is_x_symmetric": is_x_symmetric,
                    "is_y_symmetric": is_y_symmetric
                }
            }
            final_layout_block.update(nodes_dictionary)
            self.database[current_target] = final_layout_block

        except ValueError:
            messagebox.showerror("Error", "Check data table fields. Coordinates must be numeric.")
            return

        self.current_idx += 1
        self.load_active_option()

    def export_database_to_file(self, JSON_PATH):
        filename = JSON_PATH

        with open(filename, "w") as f:
            json.dump(self.database, f, indent=4)
        
        messagebox.showinfo("Done!", f"Database successfully written with graphic annotations mapping data: {filename}")
        self.root.destroy()

if __name__ == "__main__":
    if MODE == "ugv":
        IMAGE_PATH = "create_xml/images/6_ugv_config.jpeg"
        GRID_MAPPING = {
            "line": (19, 16, 305, 382),
            "diagonal": (380, 14, 665, 385),
            "triangle": (745, 15, 1028, 382),
            "rectangle-same": (19, 410, 327, 785),
            "rectangle-opposite": (382, 405, 705, 792),
            "rhombus-ext": (737, 408, 1058, 800),
            "rhombus-int": (1084, 410, 1405, 793),
            "parallelepiped-ext": (1435, 401, 1751, 780),
            "parallelepiped-int": (17, 828, 354, 1208),
            "trapezoid-ext": (385, 827, 708, 1203),
            "trapezoid-int": (747, 826, 1071, 1207),
            "pentagon": (6, 1238, 330, 1643),
            "arrow": (360, 1235, 668, 1638),
            "cross": (695, 1232, 1010, 1634),
            "central-trapezoid": (1029, 1231, 1362, 1643),
            "wave": (1379, 1232, 1711, 1643),
            "base-trapezoid": (11, 1670, 300, 2038),
            "central-parallelepiped": (348, 1649, 687, 2050),
            "2X3": (6, 2070, 284, 2457),
            "two-triangles": (340, 2068, 627, 2458),
            "vertical-parallelepiped": (650, 2064, 1022, 2473),
            "pyramid": (1040, 2065, 1328, 2455),
            "horizzontal-parallelepiped": (1351, 2065, 1756, 2443),
            "3X2": (2, 2492, 288, 2887),
            "hexagon": (320, 2498, 617, 2876),
        }
    elif MODE == "uav":
        IMAGE_PATH = "create_xml/images/3_uav_config.png"
        GRID_MAPPING = {
            "point": (24, 14, 203, 177),
            "aligned": (230, 11, 426, 175),
            "disaligned": (444, 6, 669, 179),
            "line": (16, 215, 203, 364),
            "triangle": (240, 213, 432, 366),
            "diagonal": (478, 192, 699, 371),
        }
    else:
        print("Error: no possible mode selected")
        exit()
    json_path = f"create_xml/database/{MODE}_configuration_database.json"
    root = tk.Tk()
    app = AnnotatedDatabaseBuilder(MODE, root, IMAGE_PATH, GRID_MAPPING, json_path)
    root.mainloop()