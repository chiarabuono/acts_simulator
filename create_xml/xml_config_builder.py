"""
Non-interactive xml file generator called by `analytical_ground_screening` for the best performing architectures
"""

import os
import re
from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional

from config_params import MUJOCO_TEMPLATE

class RoutingValidationError(Exception):
    """Raised when a candidate configuration violates capacity or duplicate-path constraints."""


@dataclass
class UGVUAVConfig:
    """One fully-specified configuration, equivalent to one completed run of the wizard."""
    pay_layout: str
    gnd_layout: str
    uav_layout: str
    # routing[cable_num] = {"payload_anchor": letter, "ground_anchor": letter}, cable_num in 4..9
    routing: Dict[int, Dict[str, str]]
    mirror_gnd_x: bool = False
    mirror_gnd_y: bool = False
    scale_mode: str = "Normal"       # "Small" | "Normal" | "Large"
    scale_factor: float = 1.0        # only used if scale_mode != "Normal"


def _validate_routing(cfg: UGVUAVConfig, ugv_geo_db: dict):
    """Mirrors the duplicate-path / max_cables checks from process_final_xml_data."""
    payload_connection_counts, ground_connection_counts = {}, {}
    seen_routing_pairs = set()
    errors = []

    for num, vars_dict in cfg.routing.items():
        p_node, g_node = vars_dict["payload_anchor"], vars_dict["ground_anchor"]

        if (p_node, g_node) in seen_routing_pairs:
            errors.append(f"Cable {num} duplicate path: '{p_node}' <-> '{g_node}'")
        else:
            seen_routing_pairs.add((p_node, g_node))

        payload_connection_counts[p_node] = payload_connection_counts.get(p_node, 0) + 1
        ground_connection_counts[g_node] = ground_connection_counts.get(g_node, 0) + 1

    for nodes_dict, db_source, label in [
        (payload_connection_counts, ugv_geo_db[cfg.pay_layout], "Payload"),
        (ground_connection_counts, ugv_geo_db[cfg.gnd_layout], "Ground"),
    ]:
        for letter, count in nodes_dict.items():
            if letter == "symmetry_metadata":
                continue
            max_allowed = db_source[letter].get("max_cables", 999)
            if count > max_allowed:
                errors.append(f"{label} node '{letter}': connected {count} (max {max_allowed})")

    if errors:
        raise RoutingValidationError("; ".join(errors))


def build_xml(cfg: UGVUAVConfig, ugv_geo_db: dict, uav_geo_db: dict) -> str:
    """
    Pure function: config + geometry DBs -> XML string.
    Raises RoutingValidationError if the config is infeasible (duplicate path / capacity).
    """
    _validate_routing(cfg, ugv_geo_db)

    if cfg.scale_mode in ("Small", "Large") and cfg.scale_factor <= 0:
        raise ValueError("scale_factor must be a positive number when scale_mode is Small/Large")

    payload_sites = ground_sites = tendon_elements = actuator_elements = sensor_elements = ""

    gnd_mx = -1.0 if cfg.mirror_gnd_x else 1.0
    gnd_my = -1.0 if cfg.mirror_gnd_y else 1.0

    # --- ground cables 4-9 ---
    for num, vars_dict in cfg.routing.items():
        p_node, g_node = vars_dict["payload_anchor"], vars_dict["ground_anchor"]

        px, py, _ = ugv_geo_db[cfg.pay_layout][p_node]["coords"]
        raw_gx, raw_gy, _ = ugv_geo_db[cfg.gnd_layout][g_node]["coords"]

        if cfg.scale_mode == "Small":
            gx, gy = raw_gx / cfg.scale_factor, raw_gy / cfg.scale_factor
        elif cfg.scale_mode == "Large":
            gx, gy = raw_gx * cfg.scale_factor, raw_gy * cfg.scale_factor
        else:
            gx, gy = raw_gx, raw_gy

        gx *= gnd_mx
        gy *= gnd_my
        gz = 0.0

        payload_sites += f'            <site name="hook_{num}" pos="{px} {py} -0.10" size="0.06" rgba="1 0.2 0.2 1"/>\n'
        ground_sites += f'        <site name="ground_anchor_{num}" pos="{gx} {gy} {gz}" size="0.07" rgba="1 0.2 0.2 1"/>\n'
        tendon_elements += (
            f'        <spatial name="cable_{num}" limited="true" range="0 40.0" width="0.015" rgba="1 0.3 0.3 1">\n'
            f'            <site site="ground_anchor_{num}"/>\n            <site site="hook_{num}"/>\n        </spatial>\n'
        )
        actuator_elements += f'      <position name="cable_{num}_winch" tendon="cable_{num}" kp="2000" kv="150" ctrlrange="0 40.0"/>\n'
        sensor_elements += f'    <tendonlimitfrc name="cable_{num}_tension" tendon="cable_{num}"/>\n'

    # --- UAV bodies/tendons, identical layout logic to the wizard (including its
    #     assumption that a node hosts at most 3 cables via the 1/2/3 offset branches) ---
    uav_bodies_string = uav_tendons_string = uav_actuators_string = uav_sensors_string = ""

    cables = {
        letter: uav_geo_db[cfg.uav_layout][letter]["max_cables"]
        for letter in uav_geo_db[cfg.uav_layout]
        if letter != "symmetry_metadata"
    }

    drone_idx = 0
    for letter in cables:
        raw_px, raw_py, _ = uav_geo_db[cfg.uav_layout][letter]["coords"]

        while cables[letter] != 0:
            if cables[letter] == 1:
                ux, uy = raw_px + 0.20, raw_py + 0.20
            elif cables[letter] == 2:
                ux, uy = raw_px - 0.20, raw_py - 0.20
            elif cables[letter] == 3:
                ux, uy = raw_px + 0.20, raw_py - 0.20
            uz = 0.25
            drone_idx += 1

            payload_sites += f'       <site name="hook_{drone_idx}" pos="{raw_px} {raw_py} 0.10" size="0.04" rgba="1 1 0 1"/>\n'
            cables[letter] -= 1

            uav_bodies_string += (
                f'        <body name="drone_{drone_idx}" pos="{ux:.3f} {uy:.3f} {uz:.3f}">\n'
                f'            <freejoint name="drone_{drone_idx}_joint"/>\n'
                f'            <inertial pos="0 0 0" mass="2.0" diaginertia="0.01 0.01 0.015"/>\n'
                f'            <geom name="drone_{drone_idx}_geom" type="cylinder" size="0.15 0.05" rgba="0 0.7 0.9 1" '
                f'mass="2.0" condim="3" friction="1 0.005 0.0001"/>\n'
                f'            <site name="drone_{drone_idx}_com" pos="0 0 0" size="0.02" rgba="1 1 0 1"/>\n'
                f'        </body>\n'
            )
            uav_tendons_string += (
                f'        <spatial name="cable_{drone_idx}" limited="true" range="0 1.5" width="0.015" rgba="0 0.8 0 1">\n'
                f'            <site site="drone_{drone_idx}_com"/>\n            <site site="hook_{drone_idx}"/>\n        </spatial>\n'
            )
            uav_actuators_string += (
                f'      <motor name="drone_{drone_idx}_thrust" site="drone_{drone_idx}_com" gear="0 0 1 0 0 0"/>\n'
                f'      <motor name="drone_{drone_idx}_roll"   site="drone_{drone_idx}_com" gear="0 0 0 1 0 0"/>\n'
                f'      <motor name="drone_{drone_idx}_pitch"  site="drone_{drone_idx}_com" gear="0 0 0 0 1 0"/>\n'
                f'      <motor name="drone_{drone_idx}_yaw"    site="drone_{drone_idx}_com" gear="0 0 0 0 0 1"/>\n'
            )
            uav_sensors_string += f'    <tendonlimitfrc name="cable_{drone_idx}_tension" tendon="cable_{drone_idx}"/>\n'

    return MUJOCO_TEMPLATE.format(
        payload_sites=payload_sites, uav_bodies_string=uav_bodies_string, ground_sites=ground_sites,
        uav_tendons_string=uav_tendons_string, tendon_elements=tendon_elements,
        uav_actuators_string=uav_actuators_string, actuator_elements=actuator_elements,
        uav_sensors_string=uav_sensors_string, sensor_elements=sensor_elements,
    )


def _coords_fingerprint(cfg: UGVUAVConfig, ugv_geo_db: dict):
    """Same dedup fingerprint used by the wizard, so batch runs share the dedup logic."""
    fp = []
    for vars_dict in cfg.routing.values():
        p_node, g_node = vars_dict["payload_anchor"], vars_dict["ground_anchor"]
        px, py, _ = ugv_geo_db[cfg.pay_layout][p_node]["coords"]
        gx, gy, gz = ugv_geo_db[cfg.gnd_layout][g_node]["coords"]
        fp.append((f"{px} {py} -0.10", f"{gx} {gy} {gz}"))
    fp.sort()
    return fp


def save_xml(cfg: UGVUAVConfig, xml_content: str, ugv_geo_db: dict, out_dir: str = "mujoco") -> Tuple[str, Optional[str]]:
    """
    Writes xml_content to disk using the same naming/dedup convention as the wizard.
    Returns (path_written_or_existing, message). message is None on a fresh write.
    """
    os.makedirs(out_dir, exist_ok=True)
    base_name = os.path.join(out_dir, f"{cfg.pay_layout.upper()}-{cfg.gnd_layout.upper()}-{cfg.uav_layout.upper()}")

    if cfg.scale_mode == "Small":
        base_name += f"-small-{cfg.scale_factor}".replace(".", "_")
    elif cfg.scale_mode == "Large":
        base_name += f"-large-{cfg.scale_factor}".replace(".", "_")

    if cfg.mirror_gnd_x and cfg.mirror_gnd_y:
        base_name += "-mirrorred-xy"
    elif cfg.mirror_gnd_x:
        base_name += "-mirrorred-x"
    elif cfg.mirror_gnd_y:
        base_name += "-mirrorred-y"

    current_fp = _coords_fingerprint(cfg, ugv_geo_db)
    output_filename = f"{base_name}.xml"
    counter = 2

    while os.path.exists(output_filename):
        existing_fp = []
        try:
            with open(output_filename, "r") as ef:
                content = ef.read()
            for cable_num in range(4, 10):
                p_match = re.search(rf'<site name="hook_{cable_num}" pos="([^"]+)"', content)
                g_match = re.search(rf'<site name="ground_anchor_{cable_num}" pos="([^"]+)"', content)
                if p_match and g_match:
                    existing_fp.append((p_match.group(1).strip(), g_match.group(1).strip()))
        except IOError:
            existing_fp = None

        if existing_fp:
            existing_fp.sort()

        if existing_fp == current_fp:
            return output_filename, f"Identical topology already exists at '{output_filename}'; not rewritten."

        output_filename = f"{base_name}-{counter}.xml"
        counter += 1

    with open(output_filename, "w") as f:
        f.write(xml_content)
    return output_filename, None
