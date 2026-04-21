import math
import numpy as np
import json
import os
import xacro
from xml.dom.minidom import parseString

def solve_panel_pose_and_cables(drones):
    """
    drones: list of dicts, one per drone:
        {
          'id': int,
          'drone_xyz_world': [x, y, z],   # drone position in world
          'attach_xyz_panel': [x, y, z],  # attachment point in panel local frame
          'length': float,                 # cable length
        }
    
    Returns:
        panel_xyz_world: [x, y, z]
        panel_rpy_world: [r, p, y]
        per_drone cable orientations: list of (roll, pitch, yaw) in panel local frame
    """

    # Step 1: compute cable top positions in world frame
    # assuming straight-up hang for spawn initialization
    a_world = []
    q_local = []
    for d in drones:
        drone_w = np.array(d['drone_xyz_world'])
        top_w   = drone_w + np.array([0.0, 0.0, d['length']])
        a_world.append(top_w)
        q_local.append(np.array(d['attach_xyz_panel']))

    a_world = np.array(a_world)  # (N, 3)
    q_local = np.array(q_local)  # (N, 3)

    # Step 2: find R, t via SVD (Kabsch algorithm)
    centroid_a = a_world.mean(axis=0)
    centroid_q = q_local.mean(axis=0)

    A = a_world - centroid_a  # (N, 3)
    Q = q_local - centroid_q  # (N, 3)

    H = Q.T @ A               # (3, 3)
    U, S, Vt = np.linalg.svd(H)
    
    # Ensure proper rotation (det = +1, not reflection)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T        # world_R_panel

    t = centroid_a - R @ centroid_q  # panel origin in world

    # Step 3: extract RPY from R
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2( R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2( R[1,0], R[0,0])
    else:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0.0

    # Step 4: compute cable orientations in panel local frame
    cable_rpys = []
    for d in drones:
        drone_w   = np.array(d['drone_xyz_world'])
        attach_w  = R @ np.array(d['attach_xyz_panel']) + t  # attachment in world

        # vector from attachment to drone in world frame
        v_world = drone_w - attach_w
        # express in panel local frame
        v_local = R.T @ v_world

        horiz       = math.sqrt(v_local[0]**2 + v_local[1]**2)
        cable_pitch = math.atan2(horiz, -v_local[2])
        cable_yaw   = math.atan2(v_local[1], v_local[0])
        cable_rpys.append((0.0, cable_pitch, cable_yaw))

    return list(t), [roll, pitch, yaw], cable_rpys


def get_drone_spawn_data(config_file_path):
    with open(config_file_path, 'r') as f:
        data = json.load(f)
    
    drones = data['drones']
    
    # Solve for the poses
    panel_xyz, panel_rpy, cable_rpys = solve_panel_pose_and_cables(drones)
    
    return drones, panel_xyz, panel_rpy, cable_rpys

def send_actsInfo_toxacro(xacro_file, p_xyz, p_rpy, drones, cable_rpys):
    xacro_mappings = {
        'panel_x': str(p_xyz[0]),
        'panel_y': str(p_xyz[1]),
        'panel_z': str(p_xyz[2]),
        'panel_R': str(p_rpy[0]),
        'panel_P': str(p_rpy[1]),
        'panel_Y': str(p_rpy[2]),
    }

    # TODO: send panel info to xacro

    for i, (drone, (cr, cp, cy)) in enumerate(zip(drones, cable_rpys)):
        prefix = f"drone{drone['id']}_"
        xacro_mappings.update({
            f'{prefix}id' : str(drone['id']), 
            f'{prefix}x': str(drone['drone_xyz_world'][0]),
            f'{prefix}y': str(drone['drone_xyz_world'][1]),
            f'{prefix}z': str(drone['drone_xyz_world'][2]),
            f'{prefix}attach_x': str(drone['attach_xyz_panel'][0]),
            f'{prefix}attach_y': str(drone['attach_xyz_panel'][1]),
            f'{prefix}attach_z': str(drone['attach_xyz_panel'][2]),
            f'{prefix}panel_link': str(drone['panel_link']),
            f'{prefix}len': str(drone['length']),
            f'{prefix}roll': str(cr),
            f'{prefix}pitch': str(cp),
            f'{prefix}yaw': str(cy),
        })

    robot_desc = xacro.process_file(xacro_file, mappings=xacro_mappings).toxml()
    return robot_desc

def create_actsXacro_file(pkg_share, p_xyz, p_rpy, drones, cable_rpys):
    xacro_content = f"""<?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="acts_system">
        <xacro:include filename="{os.path.join(pkg_share, 'urdf', 'panel.urdf.xacro')}" />
        <xacro:include filename="{os.path.join(pkg_share, 'urdf', 'uav_cable_assembly.urdf.xacro')}" />

    
        <xacro:panel_platform 
            prefix="main_panel_" 
            mass="0.001" 
            radius="0.7" 
            height="0.2" />
    """
    # TODO: receive panel info from the json
    #    xacro_mappings = {
    #    'panel_x': str(p_xyz[0]),
    #    'panel_y': str(p_xyz[1]),
    #    'panel_z': str(p_xyz[2]),
    #    'panel_R': str(p_rpy[0]),
    #    'panel_P': str(p_rpy[1]),
    #    'panel_Y': str(p_rpy[2]),
    # }

    # TODO: add receiving attach_xyz_panel to the panel
    #   f'{prefix}attach_x': str(drone['attach_xyz_panel'][0]),
    #   f'{prefix}attach_y': str(drone['attach_xyz_panel'][1]),
    #   f'{prefix}attach_z': str(drone['attach_xyz_panel'][2]),

    # Loop through the drones and the calculated cable orientations
    for i, (drone, (cr, cp, cy)) in enumerate(zip(drones, cable_rpys)):
        xacro_content += f"""
        <xacro:add_drone 
            id="{drone['id']}" 
            segments="10" 
            length="{drone['length']}" 
            panel_link="{drone['panel_link']}"
            x_off="{drone['drone_xyz_world'][0]}" 
            y_off="{drone['drone_xyz_world'][1]}" 
            z_off="{drone['drone_xyz_world'][2]}"
            cable_roll="{cr}"
            cable_pitch="{cp}"
            cable_yaw="{cy}"/>"""

    xacro_content += "\n</robot>"
    doc = parseString(xacro_content)
    xacro.process_doc(doc)
    robot_desc = doc.toxml()

    return robot_desc