#!/usr/bin/env python3
import os
import subprocess as sp
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory

def compile_xacro_to_loop_sdf(package_name, xacro_relative_path):
    package_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(package_path, xacro_relative_path)
    
    # Paths for intermediate conversion steps
    temp_urdf = "/tmp/compiled_flat.urdf"
    temp_sdf = "/tmp/converted_raw.sdf"

    # 1. Expand Xacro into a flattened, full URDF string
    print("[1/4] Expanding Xacro macros...")
    urdf_xml_string = xacro.process_file(xacro_file).toxml()
    with open(temp_urdf, "w") as f:
        f.write(urdf_xml_string)

    # 2. Translate the flattened URDF into native Gazebo SDFormat
    print("[2/4] Translating URDF into raw SDF...")
    sp.run(f"gz sdf -p {temp_urdf} > {temp_sdf}", shell=True, check=True)

    # 3. Dynamic Injection: Add the closed-loop joint safely
    print("[3/4] Injecting closed-loop kinematic chain links...")
    tree = ET.parse(temp_sdf)
    root = tree.getroot()
    model_element = root.find("model")

    # Create the loop closing joint
    all_links = model_element.findall("link")
    
    for link in all_links:
        link_name = link.get("name") # e.g., "ugv1_cable", "ugv2_cable", "ugv3_cable"
        
        # Check if this link is a cable and NOT the primary anchor cable (ugv1)
        if link_name.endswith("_cable") and link_name != "ugv1_cable":
            # Extract the ID or just use the name directly
            print(f"   -> Injecting closed-loop joint connecting {link_name} to payload")
            
            # Create a unique joint name based on the current cable name
            joint_name = f"{link_name}_to_payload_joint"
            
            # Build the joint element using native SDF syntax
            loop_joint = ET.Element("joint", {"name": joint_name, "type": "fixed"})
            
            parent = ET.SubElement(loop_joint, "parent")
            parent.text = "payload"
            
            child = ET.SubElement(loop_joint, "child")
            child.text = link_name
            
            # Append this specific loop-closer joint to the model tree
            model_element.append(loop_joint)
    
    # Save out the dynamically linked model
    modified_sdf_path = "/tmp/injected_final.sdf"
    tree.write(modified_sdf_path, encoding="utf-8", xml_declaration=True)

    # 4. Loop Unroller processing
    print("[4/4] Sending to loop unroller plugin parser...")
    cmd = f'ros2 run gz_attach_links unroll_loops -f "{modified_sdf_path}" -n acts_model'
    result = sp.run(cmd, shell=True, stdout=sp.PIPE, stderr=sp.PIPE)
    
    unroller_output_string = result.stdout.decode().strip()

    if result.returncode != 0:
        raise RuntimeError(f"Loop unroller crashed: {result.stderr.decode()}")

    # ROUTING FIX: Check if the unroller returned a file path or raw XML
    if unroller_output_string.startswith('<'):
        # If it returned raw XML by any chance, save it to a file so simple_launch can read it
        fallback_path = "/tmp/unrolled_fallback.sdf"
        with open(fallback_path, "w") as f:
            f.write(unroller_output_string)
        print(f"[SUCCESS] Pipeline finished! Saved raw XML to file: {fallback_path}")
        return fallback_path
    else:
        print(f"[SUCCESS] Pipeline finished! Passing file path: {unroller_output_string}")
        return unroller_output_string