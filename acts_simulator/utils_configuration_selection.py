import os
import glob
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

def select_and_load_xml():
    # Keep track of state in a dictionary so inner callback functions can modify them
    state = {
        "target_dir": "mujoco",
        "content": None,
        "filename": None
    }

    ui = tk.Tk()
    ui.title("Select MuJoCo Configuration Target")
    ui.geometry("550x400") # Slight boost to width/height to look cleaner with the extra UI elements
    
    # --- Top Frame for Controls ---
    top_frame = ttk.Frame(ui)
    top_frame.pack(fill=tk.X, padx=15, pady=(15, 5))
    
    lbl = ttk.Label(top_frame, text="Current Directory:", font=("Arial", 10, "bold"))
    lbl.pack(side=tk.LEFT)
    
    # Label that displays the currently selected folder path
    dir_label = ttk.Label(top_frame, text=os.path.abspath(state["target_dir"]), font=("Arial", 9, "italic"), foreground="blue")
    dir_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)

    # --- UI Components initialized early so handlers can reference them ---
    list_frame = ttk.Frame(ui)
    list_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)
    
    scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL)
    listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, font=("Arial", 10))
    scrollbar.config(command=listbox.yview)
    
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # --- Function to update file listing safely ---
    def refresh_file_list():
        listbox.delete(0, tk.END)
        dir_label.config(text=os.path.abspath(state["target_dir"]))
        
        if not os.path.exists(state["target_dir"]):
            return
            
        xml_files = [os.path.basename(f) for f in glob.glob(f"{state['target_dir']}/*.xml")]
        for file in sorted(xml_files):
            listbox.insert(tk.END, file)
            
        if not xml_files:
            listbox.insert(tk.END, "  -- No .xml files found in this directory --")

    # --- Action handlers ---
    def on_browse_folder():
        chosen_dir = filedialog.askdirectory(initialdir=state["target_dir"], title="Select Target XML Folder")
        if chosen_dir:  # User didn't click cancel
            state["target_dir"] = chosen_dir
            refresh_file_list()

    def on_confirm():
        selection = listbox.curselection()
        if not selection:
            messagebox.showwarning("Selection Required", "Please highlight an XML file configuration first.")
            return
            
        chosen_file = listbox.get(selection[0])
        
        # Guard against selecting the descriptive placeholder if folder is empty
        if chosen_file.startswith("  -- "):
            return
            
        full_path = os.path.join(state["target_dir"], chosen_file)
        
        try:
            with open(full_path, "r") as f:
                state["content"] = f.read()
                state["filename"] = chosen_file.replace(".xml", "")
            ui.destroy()
        except IOError as e:
            messagebox.showerror("File Read Error", f"Could not load system data:\n{e}")

    # --- Setup Interactive Buttons ---
    btn_browse = ttk.Button(top_frame, text="Browse Folder...", command=on_browse_folder)
    btn_browse.pack(side=tk.RIGHT)

    btn_load = ttk.Button(ui, text="Load Configuration Model", command=on_confirm)
    btn_load.pack(fill=tk.X, padx=15, pady=15)
    
    listbox.bind("<Double-1>", lambda event: on_confirm())
    
    # Initialize list contents for the default directory on startup
    refresh_file_list()
    
    ui.mainloop()
    
    return state["filename"], state["content"]


def select_and_load_folder():
    # Track state in a dictionary so inner callback functions can modify them
    state = {
        "target_dir": "mujoco",
        "selected_folder_path": None,
        "selected_folder_name": None
    }

    ui = tk.Tk()
    ui.title("Select Configuration Folder Target")
    ui.geometry("550x400")
    
    # --- Top Frame for Controls ---
    top_frame = ttk.Frame(ui)
    top_frame.pack(fill=tk.X, padx=15, pady=(15, 5))
    
    lbl = ttk.Label(top_frame, text="Current Directory:", font=("Arial", 10, "bold"))
    lbl.pack(side=tk.LEFT)
    
    # Label that displays the currently selected parent folder path
    dir_label = ttk.Label(top_frame, text=os.path.abspath(state["target_dir"]), font=("Arial", 9, "italic"), foreground="blue")
    dir_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)

    # --- UI Components ---
    list_frame = ttk.Frame(ui)
    list_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)
    
    scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL)
    listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, font=("Arial", 10))
    scrollbar.config(command=listbox.yview)
    
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # --- Function to update directory listing safely ---
    def refresh_folder_list():
        listbox.delete(0, tk.END)
        dir_label.config(text=os.path.abspath(state["target_dir"]))
        
        if not os.path.exists(state["target_dir"]):
            listbox.insert(tk.END, "  -- Parent directory does not exist --")
            return
            
        # Get only subdirectories in the target path
        try:
            subdirs = [
                d for d in os.listdir(state["target_dir"]) 
                if os.path.isdir(os.path.join(state["target_dir"], d))
            ]
            
            for folder in sorted(subdirs):
                listbox.insert(tk.END, f"📁 {folder}")
                
            if not subdirs:
                listbox.insert(tk.END, "  -- No subfolders found in this directory --")
        except PermissionError:
            messagebox.showerror("Permission Error", "Cannot access this directory.")

    # --- Action handlers ---
    def on_browse_parent():
        chosen_dir = filedialog.askdirectory(initialdir=state["target_dir"], title="Select Parent Directory")
        if chosen_dir:  # User didn't click cancel
            state["target_dir"] = chosen_dir
            refresh_folder_list()

    def on_confirm():
        selection = listbox.curselection()
        if not selection:
            messagebox.showwarning("Selection Required", "Please highlight a folder first.")
            return
            
        chosen_item = listbox.get(selection[0])
        
        # Guard against selecting the descriptive placeholder
        if chosen_item.startswith("  -- "):
            return
            
        # Clean the folder icon prefix '📁 ' off the string name
        folder_name = chosen_item.replace("📁 ", "").strip()
        full_path = os.path.join(state["target_dir"], folder_name)
        
        state["selected_folder_name"] = folder_name
        state["selected_folder_path"] = full_path
        
        ui.destroy()

    # --- Setup Interactive Buttons ---
    btn_browse = ttk.Button(top_frame, text="Browse Parent...", command=on_browse_parent)
    btn_browse.pack(side=tk.RIGHT)

    btn_load = ttk.Button(ui, text="Select Highlighted Folder", command=on_confirm)
    btn_load.pack(fill=tk.X, padx=15, pady=15)
    
    # Double click selects the folder
    listbox.bind("<Double-1>", lambda event: on_confirm())
    
    # Initialize list contents for the default directory on startup
    refresh_folder_list()
    
    ui.mainloop()
    
    return state["selected_folder_name"], state["selected_folder_path"]