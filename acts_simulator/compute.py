import math

def compute_cable_properties(p_start, p_end, cable_type="ugv"):
    """
    Computes the roll, pitch, yaw angles, and the exact straight-line length 
    needed for a flexible xacro cable to connect two 3D points.

    Parameters:
    -----------
    p_start : tuple or list
        The 3D coordinates (x, y, z) of the cable origin (e.g., panel attachment).
    p_end : tuple or list
        The 3D coordinates (x, y, z) of the target destination (e.g., ground hook).
    cable_type : str
        The macro mode configuration ('ugv' or 'uav').

    Returns:
    --------
    dict
        A dictionary containing the calculated roll, pitch, yaw, and total length.
    """
    # 1. Calculate relative differences
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    dz = p_end[2] - p_start[2]

    # 2. Compute Length (Euclidean 3D Distance)
    total_length = math.sqrt(dx**2 + dy**2 + dz**2)

    # 3. Compute Yaw (Rotation around global Z-axis)
    cable_yaw = math.atan2(dy, dx)

    # 4. Compute horizontal travel distance (hypotenuse on XY plane)
    r = math.sqrt(dx**2 + dy**2)

    # 5. Compute Pitch (Tilt away from the vertical axis)
    if cable_type == "ugv":
        # UGV cables grow downwards (negative Z scope)
        cable_pitch = math.atan2(r, abs(dz))
    else:
        # UAV cables grow upwards (positive Z scope)
        cable_pitch = math.atan2(r, dz)

    # 6. Roll is 0 for pointing task
    cable_roll = 0.0

    return {
        "roll": round(cable_roll, 4),
        "pitch": round(cable_pitch, 4),
        "yaw": round(cable_yaw, 4),
        "length": round(total_length, 4)
    }

def compute_payload_coordinates(point):
    COMcoordinate = (0, 0, 1.0)
    return (point[0] - COMcoordinate[0], point[1] - COMcoordinate[1], point[2] - COMcoordinate[2])

def write_intro(fileName):
    with open(fileName, "w") as f:
        f.write(f"<?xml version='1.0'?> \n")
        f.write(f"<robot xmlns:xacro='http://www.ros.org/wiki/xacro' name='acts'> \n")
        f.write(f"<xacro:include filename='components/panel.xacro' /> \n")
        f.write(f"<xacro:include filename='uav_cable_assembly.urdf.xacro' /> \n")
        f.write(f"<xacro:include filename='ugv_cable_assembly.urdf.xacro' /> \n")
        f.write(f"<xacro:include filename='components/hook.xacro' /> \n \n")

        f.write(f"<xacro:property name='sqrt3' value='$3**0.5'/> \n")
        f.write(f"<xacro:property name='cable_segments' value='10'/>\n")
        f.write(f"<xacro:property name='system_heigth' value='1.0'/>\n")
        f.write(f"<xacro:property name='uav_cable_len' value='1.0'/>\n \n")

        f.write(f"<link name='base_link'/>\n\n")

        f.write(f"<xacro:panel \nlink_name='chassis_link' \nmass='0.001' \nwidth='2' \nlength='2'\nheigth='0.2' \n")
        f.write(f"x='0' y='0' z='$system_heigth' \nmaterial_name='Blue' \nmaterial_color='0.0 0.0 0.8 0.0' />\n\n")     

        f.write(f"<joint name='base_to_chassis_joint' type='fixed'> \n<parent link='base_link'/> \n<child link='chassis_link'/> \n<origin xyz='0 0 0.0' rpy='0 0 0'/> \n</joint> \n\n")     

def write_end(fileName):
    with open(fileName, "a") as f:
        f.write(f"</robot>")

def write_ugv(start_points, end_points, fileName):
    letters = ["A", "B", "C", "D", "E", "F"]
    
    for i in range(len(start_points)):
        props = compute_cable_properties(start_points[i], end_points[i], cable_type="ugv")
        ref_sys = start_points[i] #compute_payload_coordinates(start_points[i])
    
        print(f"--- Calculated Xacro Properties cable  {i} ---")
        print(f"Required Length: {props['length']} meters")
        print(f"cable_roll:      {props['roll']} rad")
        print(f"cable_pitch:     {props['pitch']} rad")
        print(f"cable_yaw:       {props['yaw']} rad")
        print(f"hook point coordinates {start_points[i]} [INERTIAL FRAME]")
        print(f"hook point coordinates {ref_sys} [PAYLOAD FRAME]")

        with open(fileName, "a") as f:
            f.write(f"<xacro:property name='hook{letters[i]}_x' value='{ref_sys[0]}'/> \n")   
            f.write(f"<xacro:property name='hook{letters[i]}_y' value='{ref_sys[1]}'/> \n") 
            f.write(f"<xacro:property name='hook{letters[i]}_z' value='-0.10'/> \n \n")

            f.write(f"<xacro:property name='anchor{letters[i]}_x' value='{end_points[i][0]}'/> \n")   
            f.write(f"<xacro:property name='anchor{letters[i]}_y' value='{end_points[i][1]}'/> \n") 
            f.write(f"<xacro:property name='anchor{letters[i]}_z' value='0.0'/> \n \n")

            f.write(f"<xacro:ground_hook prefix='ugv{i+1}_' anchorX='anchor{letters[i]}_x' anchorY='anchor{letters[i]}_y' anchorZ='anchor{letters[i]}_z' /> \n \n")

            f.write(f"<xacro:add_ugv \n")
            f.write(f"id='{i+1}'\n") 
            f.write(f"segments='$cable_segments' \n")
            f.write(f"length='{props['length']}' \n")
            f.write(f"panel_link='chassis_link' \n")
            f.write(f"x_off='{ref_sys[0]}' \n")
            f.write(f"y_off='{ref_sys[1]}' \n")
            f.write(f"z_off='{ref_sys[2]}' \n")
            f.write(f"cable_roll='{props['roll']}' cable_pitch='{-props['pitch']}' cable_yaw='{props['yaw']}'/> \n \n")

def write_uav(hooks, fileName):
    for i in range(len(hooks)):
        with open(fileName, "a") as f:
            f.write(f"<xacro:property name='hook{i+1}_x' value='{hooks[i][0]}'/>\n")
            f.write(f"<xacro:property name='hook{i+1}_y' value='{hooks[i][1]}'/>\n")
            f.write(f"<xacro:property name='hook{i+1}_z' value='{hooks[i][2]}'/>\n \n")
            f.write(f"<xacro:add_drone \n")
            f.write(f"id='{i+1}' \n")
            f.write(f"segments='$cable_segments' \n")
            f.write(f"length='$uav_cable_len' \n")
            f.write(f"panel_link='chassis_link'\n")
            f.write(f"x_off='hook{i+1}_x' \n")
            f.write(f"y_off='hook{i+1}_y' \n")
            f.write(f"z_off='hook{i+1}_z+system_heigth' \n ")
            f.write(f"cable_roll='0' cable_pitch='0' cable_yaw='0' /> \n \n")

def adjust_file(fileName):
    with open(fileName, "r") as f:
        data = f.readlines()

    with open(fileName, "w") as f:
        for line in data:
            modified_line = (
                line.replace("$cable_segments", "${cable_segments}")
                    .replace("$system_heigth'", "${system_heigth}'")
                    .replace("$uav_cable_len", "${uav_cable_len}")
                    .replace("+system_heigth'", "+system_heigth}'")
                    .replace("'$3**0.5'", "'${3**0.5}'")
            )

            if "<xacro:property" not in modified_line:
                modified_line = (modified_line.replace("'hook", "'${hook")
                                                .replace("_x'", "_x}'")
                                                .replace("_y'", "_y}'")
                                                .replace("_z'", "_z}'")
                                                .replace("'anchor", "'${anchor")
                    )
            
            f.write(modified_line)


if __name__ == "__main__":
    a = 0.9
    b = 0.9
    numberIdecide1 = 0.5
    numberIdecide2 = 0.5
    c = b + numberIdecide1
    d = a + numberIdecide2
    z = 0.9

    start_points = [
        (-a, b, z), # 1
        (0, b, z), # 2
        (a, b, z), # 3
        (-a, -b, z), # 4
        (0, -b, z), # 5
        (a, -b, z)  # 6
    ]

    end_points = [
        (-d, c, 0), # 1
        (0, c, 0), # 2
        (d, c, 0), # 3
        (-d, -c, 0), # 4
        (0, -c, 0), # 5
        (d, -c, 0)  # 6
    ]

    hooks = [
        (0.0, "${sqrt3/3}", 0.10), #1
        (-0.5, "${-sqrt3 / 6}", 0.10), #2
        (0.5, "${-sqrt3 / 6}", 0.10) #3
    ]

    file = "urdf/acts_model.xacro"
    write_intro(file)
    write_ugv(start_points, end_points, file)
    write_uav(hooks, file)
    write_end(file)
    adjust_file(file)
