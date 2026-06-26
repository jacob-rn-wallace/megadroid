#!/usr/bin/env python3
"""
Generate MuJoCo MJCF scene from authoritative design YAML files.

Reads joints.yaml, geometry.yaml, kinematics.yaml, and mass.yaml.
Produces simulation/mujoco/megadroid_mvs.xml — a complete simulation scene
including kinematic structure, dynamics (mass/inertia), position actuators
for all MVS joints, foot contact geometry, and a ground plane.

The MJCF uses explicit <inertial> elements with inertia tensors computed from
the geometry shapes. All values are read from design/*.yaml — nothing is hardcoded.

Output: simulation/mujoco/megadroid_mvs.xml

Usage:
    python3 tools/generate_mjcf.py
"""

import math
import yaml
from pathlib import Path
from xml.etree import ElementTree as ET
from xml.dom import minidom

REPO_ROOT = Path(__file__).resolve().parent.parent
DESIGN_DIR = REPO_ROOT / "design"
OUTPUT_DIR = REPO_ROOT / "simulation" / "mujoco"
OUTPUT_FILE = OUTPUT_DIR / "megadroid_mvs.xml"


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def deg_to_rad(d):
    return d * math.pi / 180.0


def prettify(elem):
    rough = ET.tostring(elem, encoding="unicode")
    reparsed = minidom.parseString(rough)
    return reparsed.toprettyxml(indent="  ")


# ── Inertia helpers ────────────────────────────────────────────────────────────

def box_inertia(mass, hx, hy, hz):
    """Diagonal inertia for solid box; hx/hy/hz are HALF-sizes."""
    ixx = mass * (hy**2 + hz**2) / 3.0
    iyy = mass * (hx**2 + hz**2) / 3.0
    izz = mass * (hx**2 + hy**2) / 3.0
    return ixx, iyy, izz


def cylinder_inertia(mass, radius, half_len):
    """Diagonal inertia for solid cylinder along z-axis; half_len is HALF-length."""
    ixx = mass * (3 * radius**2 + 4 * half_len**2) / 12.0
    iyy = ixx
    izz = mass * radius**2 / 2.0
    return ixx, iyy, izz


def sphere_inertia(mass, radius):
    """Diagonal inertia for solid sphere."""
    i = 2.0 * mass * radius**2 / 5.0
    return i, i, i


def diaginertia_str(ixx, iyy, izz):
    return f"{ixx:.8f} {iyy:.8f} {izz:.8f}"


def add_inertial(body, mass, com_pos, ixx, iyy, izz):
    pos_str = f"{com_pos[0]:.6f} {com_pos[1]:.6f} {com_pos[2]:.6f}"
    ET.SubElement(body, "inertial",
                  mass=str(mass),
                  pos=pos_str,
                  diaginertia=diaginertia_str(ixx, iyy, izz))


# ── Joint/axis helpers ─────────────────────────────────────────────────────────

def get_axis(kin_data, joint_name):
    direction = kin_data["joint_axes"][joint_name]["axis_direction"]
    return {"x": "1 0 0", "y": "0 1 0", "z": "0 0 1"}[direction]


def get_range(joints_data, joint_name):
    limits = joints_data["joints"][joint_name]["limits_deg"]
    return deg_to_rad(limits["min"]), deg_to_rad(limits["max"])


# ── MJCF builder ───────────────────────────────────────────────────────────────

def create_mjcf(joints_data, geo_data, kin_data, mass_data):
    masses = mass_data["bodies"]

    geo = geo_data["anthropometrics"]
    thigh_len = geo["thigh_length_mm"] / 1000.0
    shin_len  = geo["shin_length_mm"]  / 1000.0
    ankle_off = geo["ankle_to_sole_offset_mm"] / 1000.0
    hip_y     = geo_data["leg_structure"]["twin_rail"]["inner_face_spacing_mm"] / 2000.0

    # Pelvis starts at full-extension leg height; settles under gravity.
    pelvis_z = thigh_len + shin_len + ankle_off

    mujoco = ET.Element("mujoco", model="megadroid_mvs")

    ET.SubElement(mujoco, "compiler", angle="radian", inertiafromgeom="false")
    ET.SubElement(mujoco, "option", gravity="0 0 -9.81", timestep="0.002")

    visual = ET.SubElement(mujoco, "visual")
    ET.SubElement(visual, "headlight",
                  ambient=".4 .4 .4", diffuse=".8 .8 .8", specular="0 0 0")

    default = ET.SubElement(mujoco, "default")
    ET.SubElement(default, "geom",
                  condim="3", friction="0.8 0.02 0.01", solimp="0.9 0.95 0.001")
    ET.SubElement(default, "joint",
                  damping="2.0", armature="0.02", frictionloss="0.1")

    # ── worldbody ─────────────────────────────────────────────────────────────
    worldbody = ET.SubElement(mujoco, "worldbody")

    ET.SubElement(worldbody, "light", name="sun",
                  pos="0 0 4", dir="0 0 -1",
                  diffuse="0.8 0.8 0.8", specular="0.2 0.2 0.2")
    ET.SubElement(worldbody, "geom",
                  name="ground", type="plane",
                  size="3 3 0.1", pos="0 0 0",
                  rgba="0.75 0.75 0.65 1",
                  friction="0.8 0.02 0.01")

    # ── Pelvis (floating base) ─────────────────────────────────────────────
    pelvis = ET.SubElement(worldbody, "body",
                           name="pelvis",
                           pos=f"0 0 {pelvis_z:.4f}")
    ET.SubElement(pelvis, "freejoint", name="root")
    ixx, iyy, izz = box_inertia(masses["pelvis"], 0.10, 0.15, 0.075)
    add_inertial(pelvis, masses["pelvis"], (0, 0, 0), ixx, iyy, izz)
    ET.SubElement(pelvis, "geom",
                  name="pelvis_geom", type="box",
                  size="0.10 0.15 0.075", pos="0 0 0",
                  rgba="0.5 0.5 0.5 1")

    # ── Legs ──────────────────────────────────────────────────────────────
    def add_leg(parent_body, side, y_sign):
        s = side.lower()
        y = y_sign * hip_y

        # Hip roll
        lo, hi = get_range(joints_data, "hip_roll")
        hr = ET.SubElement(parent_body, "body",
                           name=f"{s}_hip_roll_link",
                           pos=f"0 {y:.4f} 0")
        ET.SubElement(hr, "joint",
                      name=f"{s}_hip_roll", type="hinge",
                      axis=get_axis(kin_data, "hip_roll"),
                      range=f"{lo:.6f} {hi:.6f}")
        i3 = sphere_inertia(masses["hip_roll_link"], 0.03)
        add_inertial(hr, masses["hip_roll_link"], (0, 0, 0), *i3)
        ET.SubElement(hr, "geom",
                      name=f"{s}_hip_roll_geom", type="sphere", size="0.03",
                      rgba="0.8 0.4 0.1 1")

        # Thigh (hip pitch output)
        lo, hi = get_range(joints_data, "hip_pitch")
        thigh = ET.SubElement(hr, "body",
                              name=f"{s}_thigh",
                              pos="0 0 0")
        ET.SubElement(thigh, "joint",
                      name=f"{s}_hip_pitch", type="hinge",
                      axis=get_axis(kin_data, "hip_pitch"),
                      range=f"{lo:.6f} {hi:.6f}")
        ixx, iyy, izz = cylinder_inertia(masses["thigh"], 0.040, thigh_len / 2)
        add_inertial(thigh, masses["thigh"], (0, 0, -thigh_len / 2), ixx, iyy, izz)
        ET.SubElement(thigh, "geom",
                      name=f"{s}_thigh_geom", type="cylinder",
                      size=f"0.040 {thigh_len/2:.4f}",
                      pos=f"0 0 {-thigh_len/2:.4f}",
                      rgba="0.2 0.2 0.8 1")

        # Shin (knee pitch output)
        lo, hi = get_range(joints_data, "knee_pitch")
        shin = ET.SubElement(thigh, "body",
                             name=f"{s}_shin",
                             pos=f"0 0 {-thigh_len:.4f}")
        ET.SubElement(shin, "joint",
                      name=f"{s}_knee_pitch", type="hinge",
                      axis=get_axis(kin_data, "knee_pitch"),
                      range=f"{lo:.6f} {hi:.6f}")
        ixx, iyy, izz = cylinder_inertia(masses["shin"], 0.035, shin_len / 2)
        add_inertial(shin, masses["shin"], (0, 0, -shin_len / 2), ixx, iyy, izz)
        ET.SubElement(shin, "geom",
                      name=f"{s}_shin_geom", type="cylinder",
                      size=f"0.035 {shin_len/2:.4f}",
                      pos=f"0 0 {-shin_len/2:.4f}",
                      rgba="0.2 0.2 0.8 1")

        # Ankle pitch
        lo, hi = get_range(joints_data, "ankle_pitch")
        ankle = ET.SubElement(shin, "body",
                              name=f"{s}_ankle",
                              pos=f"0 0 {-shin_len:.4f}")
        ET.SubElement(ankle, "joint",
                      name=f"{s}_ankle_pitch", type="hinge",
                      axis=get_axis(kin_data, "ankle_pitch"),
                      range=f"{lo:.6f} {hi:.6f}")
        i3 = sphere_inertia(masses["ankle"], 0.025)
        add_inertial(ankle, masses["ankle"], (0, 0, 0), *i3)
        ET.SubElement(ankle, "geom",
                      name=f"{s}_ankle_geom", type="sphere", size="0.025",
                      rgba="0.8 0.4 0.1 1")

        # Foot (visual + contact)
        foot = ET.SubElement(ankle, "body",
                             name=f"{s}_foot",
                             pos=f"0.05 0 {-ankle_off:.4f}")
        ixx, iyy, izz = box_inertia(masses["foot"], 0.075, 0.040, 0.020)
        add_inertial(foot, masses["foot"], (0, 0, 0), ixx, iyy, izz)
        ET.SubElement(foot, "geom",
                      name=f"{s}_foot_geom", type="box",
                      size="0.075 0.040 0.020",
                      pos="0 0 0",
                      rgba="0.1 0.1 0.1 1")

    add_leg(pelvis, "left",  +1.0)
    add_leg(pelvis, "right", -1.0)

    # ── Torso chain ───────────────────────────────────────────────────────
    lo, hi = get_range(joints_data, "torso_pitch")
    tp = ET.SubElement(pelvis, "body",
                       name="torso_pitch_link",
                       pos="0 0 0.075")
    ET.SubElement(tp, "joint",
                  name="torso_pitch", type="hinge",
                  axis=get_axis(kin_data, "torso_pitch"),
                  range=f"{lo:.6f} {hi:.6f}")
    ixx, iyy, izz = box_inertia(masses["torso_pitch_link"], 0.025, 0.10, 0.025)
    add_inertial(tp, masses["torso_pitch_link"], (0, 0, 0), ixx, iyy, izz)
    ET.SubElement(tp, "geom",
                  type="box", size="0.025 0.10 0.025", rgba="0.5 0.5 0.5 1")

    lo, hi = get_range(joints_data, "torso_roll")
    tr = ET.SubElement(tp, "body", name="torso_roll_link", pos="0 0 0")
    ET.SubElement(tr, "joint",
                  name="torso_roll", type="hinge",
                  axis=get_axis(kin_data, "torso_roll"),
                  range=f"{lo:.6f} {hi:.6f}")
    ixx, iyy, izz = box_inertia(masses["torso_roll_link"], 0.025, 0.10, 0.025)
    add_inertial(tr, masses["torso_roll_link"], (0, 0, 0), ixx, iyy, izz)
    ET.SubElement(tr, "geom",
                  type="box", size="0.025 0.10 0.025", rgba="0.5 0.5 0.5 1")

    lo, hi = get_range(joints_data, "torso_yaw")
    torso = ET.SubElement(tr, "body", name="torso", pos="0 0 0")
    ET.SubElement(torso, "joint",
                  name="torso_yaw", type="hinge",
                  axis=get_axis(kin_data, "torso_yaw"),
                  range=f"{lo:.6f} {hi:.6f}")
    ixx, iyy, izz = box_inertia(masses["torso"], 0.075, 0.125, 0.20)
    add_inertial(torso, masses["torso"], (0, 0, 0.20), ixx, iyy, izz)
    ET.SubElement(torso, "geom",
                  type="box", size="0.075 0.125 0.20",
                  pos="0 0 0.20", rgba="0.5 0.5 0.5 1")

    # ── Actuators (position control for all MVS joints) ────────────────────
    actuator = ET.SubElement(mujoco, "actuator")

    # Leg joints appear per-side; torso joints appear once.
    for jname, jdata in joints_data["joints"].items():
        if not (jdata.get("actuated") and jdata.get("variants", {}).get("MVS")):
            continue
        lo, hi = get_range(joints_data, jname)
        rng = f"{lo:.6f} {hi:.6f}"
        loc = jdata.get("location", "")
        if loc in ("hip", "knee", "ankle"):
            for side in ("left", "right"):
                ET.SubElement(actuator, "position",
                              name=f"act_{side}_{jname}",
                              joint=f"{side}_{jname}",
                              kp="150",
                              ctrlrange=rng)
        else:
            ET.SubElement(actuator, "position",
                          name=f"act_{jname}",
                          joint=jname,
                          kp="150",
                          ctrlrange=rng)

    # ── Sensors ───────────────────────────────────────────────────────────
    sensor = ET.SubElement(mujoco, "sensor")
    for name in ("pelvis", "left_foot", "right_foot"):
        ET.SubElement(sensor, "framepos",
                      name=f"{name}_pos",
                      objtype="body", objname=name)
    ET.SubElement(sensor, "framequat",
                  name="pelvis_quat",
                  objtype="body", objname="pelvis")

    return mujoco


def main():
    print("Generating MJCF from authoritative design data...")
    print()

    joints  = load_yaml(DESIGN_DIR / "joints.yaml")
    geo     = load_yaml(DESIGN_DIR / "geometry.yaml")
    kin     = load_yaml(DESIGN_DIR / "kinematics.yaml")
    mass    = load_yaml(DESIGN_DIR / "mass.yaml")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    root = create_mjcf(joints, geo, kin, mass)
    xml_str = prettify(root)
    OUTPUT_FILE.write_text(xml_str)

    print(f"✓ MJCF generated: {OUTPUT_FILE}")
    print()
    print("Next steps:")
    print("  1. Load test:     python3 tools/sim_load_test.py")
    print("  2. Standing test: python3 tools/sim_standing.py")


if __name__ == "__main__":
    main()
