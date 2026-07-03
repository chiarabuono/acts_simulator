import os
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

MODE = "ugv"

class GuidedCropFinderApp:
    def __init__(self, root, img_path, options, MODE):
        self.root = root
        self.root.title("Guided Configuration Grid Mapper")
        self.root.geometry("1000x800")
        
        if not os.path.exists(img_path):
            print(f"Error: Image not found at {img_path}")
            self.root.quit()
            return

        self.orig_img = Image.open(img_path)

        self.options_to_map = options
        self.current_idx = 0
        self.mapped_dictionary = {}

        # --- TOP CONTROL BAR ---
        control_frame = ttk.Frame(root, padding=10)
        control_frame.pack(side=tk.TOP, fill=tk.X)
        
        self.lbl_instruction = ttk.Label(
            control_frame, 
            text="", 
            font=("Arial", 13, "bold"), 
            foreground="darkblue"
        )
        self.lbl_instruction.pack(side=tk.LEFT, padx=10)
        
        self.btn_skip = ttk.Button(control_frame, text="Skip Option", command=self.skip_current)
        self.btn_skip.pack(side=tk.RIGHT, padx=10)

        # --- SCROLLABLE FRAME CANVAS SETUP ---
        outer_frame = ttk.Frame(root)
        outer_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(outer_frame, scrollregion=(0, 0, self.orig_img.width, self.orig_img.height))
        vbar = ttk.Scrollbar(outer_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        hbar = ttk.Scrollbar(outer_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        
        self.canvas.config(xscrollcommand=hbar.set, yscrollcommand=vbar.set)
        
        vbar.pack(side=tk.RIGHT, fill=tk.Y)
        hbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Draw Image
        self.photo = ImageTk.PhotoImage(self.orig_img)
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

        # State tracking variables for clicking/dragging
        self.start_x = None
        self.start_y = None
        self.rect_id = None

        # Bind mouse events to the canvas
        self.canvas.bind("<ButtonPress-1>", self.on_button_press)
        self.canvas.bind("<B1-Motion>", self.on_move_press)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release)
        
        # Bind Mouse Wheel for scrolling support
        self.canvas.bind_all("<MouseWheel>", lambda e: self.canvas.yview_scroll(int(-1*(e.delta/120)), "units"))

        self.update_instruction_label()

    def update_instruction_label(self):
        if self.current_idx < len(self.options_to_map):
            current_target = self.options_to_map[self.current_idx]
            self.lbl_instruction.config(
                text=f"Step {self.current_idx + 1}/{len(self.options_to_map)}: Click & Drag a box over target: '{current_target}'"
            )
        else:
            self.lbl_instruction.config(text="All configurations mapped successfully! Check terminal output.")
            self.print_final_dictionary()
            self.save_file(MODE)
            self.root.destroy()

    def on_button_press(self, event):
        # Translate viewport mouse coordinates to absolute canvas pixel locations
        self.start_x = self.canvas.canvasx(event.x)
        self.start_y = self.canvas.canvasy(event.y)

        if self.rect_id:
            self.canvas.delete(self.rect_id)

        self.rect_id = self.canvas.create_rectangle(
            self.start_x, self.start_y, self.start_x, self.start_y, outline="red", width=2
        )

    def on_move_press(self, event):
        cur_x = self.canvas.canvasx(event.x)
        cur_y = self.canvas.canvasy(event.y)
        self.canvas.coords(self.rect_id, self.start_x, self.start_y, cur_x, cur_y)

    def on_button_release(self, event):
        end_x = self.canvas.canvasx(event.x)
        end_y = self.canvas.canvasy(event.y)

        left = int(min(self.start_x, end_x))
        top = int(min(self.start_y, end_y))
        right = int(max(self.start_x, end_x))
        bottom = int(max(self.start_y, end_y))

        # Check that it's a valid bounding frame size drag
        if (right - left) > 15 and (bottom - top) > 15:
            current_target = self.options_to_map[self.current_idx]
            self.mapped_dictionary[current_target] = (left, top, right, bottom)
            
            # Temporary confirmation box right on canvas to show progress
            self.canvas.create_rectangle(left, top, right, bottom, outline="green", width=1)
            self.canvas.create_text(left + 5, top + 12, text=current_target, fill="green", anchor=tk.W)

            # Move to next task option 
            self.current_idx += 1
            if self.rect_id:
                self.canvas.delete(self.rect_id)
            self.update_instruction_label()

    def skip_current(self):
        if self.current_idx < len(self.options_to_map):
            current_target = self.options_to_map[self.current_idx]
            print(f"Skipped mapping for: {current_target}")
            self.current_idx += 1
            self.update_instruction_label()

    def print_final_dictionary(self):
        print("\n================== GRID_MAPPING CODE DICTIONARY ==================")
        print("GRID_MAPPING = {")
        for key, value in self.mapped_dictionary.items():
            print(f"    \"{key}\": {value},")
        print("}")
        print("==================================================================\n")
        messagebox.showinfo("Done!", "The complete dictionary has been printed to your terminal window output!")

    def save_file(self, MODE):
        with open(f"{MODE}_configuration_database.json", "w") as f:
            f.write("{")
            for key, value in self.mapped_dictionary.items():
                f.write(f"    \"{key}\": {value},")
            f.write("}")



if __name__ == "__main__":
    if MODE == "ugv":
        IMAGE_PATH = "create_xml/images/6_ugv_config.jpeg"
        options = [
                "line", "diagonal", "triangle", "rectangle-same", "rectangle-opposite",
                "rhombus-ext", "rhombus-int", "parallelepiped-ext", "parallelepiped-int",
                "trapezoid-ext", "trapezoid-int", "pentagon", "arrow", "cross",
                "central-trapezoid", "wave", "base-trapezoid", "central-parallelepiped",
                "2X3", "two-triangles", "vertical-parallelepiped", "pyramid",
                "horizzontal-parallelepiped", "3X2", "hexagon"
            ]
    elif MODE == "uav":
        IMAGE_PATH = "create_xml/images/3_uav_config.png"
        options = [
                "point", "aligned", "disaligned",
                "line", "triangle", "diagonal"
            ]
    else:
        print("Error: no possible mode selected")

    root = tk.Tk()
    app = GuidedCropFinderApp(root, IMAGE_PATH, options, MODE)
    root.mainloop()