#!/usr/bin/env python3
"""
Generate URDF from authoritative design YAML files.

Reads design/joints.yaml, design/geometry.yaml, design/kinematics.yaml,
and design/mass.yaml and produces a dynamics-capable URDF.

Output: simulation/urdf/megadroid_mvs.urdf
"""

import math
import yaml
from pathlib import Path
from xml.etree import ElementTree as ET
from xml.dom import minidom

REPO_ROOT = Path(__file__).resolve().parent.parent
DESIGN_DIR = REPO_ROOT / "design"
OUTPUT_DIR = REPO_ROOT / "simulation" / "urdf"
OUTPUT_FILE = OUTPUT_DIR / "megadroid_mvs.urdf"


def load_yaml(path: Path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def prettify_xml(elem):
    """Return a pretty-printed XML string."""
    rough_string = ET.tostring(elem, encoding="unicode")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def deg_to_rad(deg):
    return deg * math.pi / 180.0


# ── Inertia helpers ────────────────────────────────────────────────────────────

def _fmt3(a, b, c):
    return f"{a:.6f} {b:.6f} {c:.6f}"


def add_inertial(link, mass, com_xyz, ixx, iyy, izz, ixy=0.0, ixz=0.0, iyz=0.0):
    """Append a <inertial> element to a URDF <link>."""
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin",
                  xyz=_fmt3(*com_xyz), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(inertial, "inertia",
                  ixx=f"{ixx:.8f}", iyy=f"{iyy:.8f}", izz=f"{izz:.8f}",
                  ixy=f"{ixy:.8f}", ixz=f"{ixz:.8f}", iyz=f"{iyz:.8f}")


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


# ── URDF helpers ───────────────────────────────────────────────────────────────

def axis_string(axis_direction: str) -> str:
    """Convert axis direction letter from kinematics.yaml to URDF xyz string."""
    mapping = {
        "x": "1 0 0",
        "y": "0 1 0",
        "z": "0 0 1",
    }
    val = mapping.get(axis_direction.lower())
    if val is None:
        raise ValueError(f"Unknown axis direction: {axis_direction!r}. Expected x, y, or z.")
    return val


def get_joint_axis(kinematics_data: dict, joint_name: str) -> str:
    """
    Read axis direction for a named joint from kinematics.yaml joint_axes block.
    Returns URDF xyz string e.g. '0 1 0'.
    """
    joint_axes = kinematics_data.get("joint_axes", {})
    entry = joint_axes.get(joint_name)
    if entry is None:
        raise ValueError(
            f"No joint_axes entry found in kinematics.yaml for joint '{joint_name}'. "
            f"Add it before regenerating the URDF."
        )
    direction = entry.get("axis_direction")
    if direction is None:
        raise ValueError(
            f"joint_axes entry for '{joint_name}' is missing 'axis_direction' field."
        )
    return axis_string(direction)


def get_limits(joints_data: dict, joint_name: str):
    """Return (lower_rad, upper_rad) for a named joint."""
    j = joints_data["joints"][joint_name]
    limits = j.get("limits_deg")
    if limits is None:
        raise ValueError(f"Joint '{joint_name}' has null limits_deg — set limits before generating URDF.")
    return deg_to_rad(limits["min"]), deg_to_rad(limits["max"])


def is_mvs_actuated(joints_data: dict, joint_name: str) -> bool:
    """Return True if the joint is actuated and included in the MVS config."""
    j = joints_data["joints"].get(joint_name, {})
    if not j.get("actuated", False):
        return False
    return j.get("variants", {}).get("MVS", False)


def add_link(robot, name, visual_origin_xyz, visual_origin_rpy, geometry, color_rgba, material_name):
    """Helper to add a link with a single visual element."""
    link = ET.SubElement(robot, "link", name=name)
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin",
                  xyz=" ".join(str(v) for v in visual_origin_xyz),
                  rpy=" ".join(str(v) for v in visual_origin_rpy))
    geom = ET.SubElement(visual, "geometry")
    geom.append(geometry)
    material = ET.SubElement(visual, "material", name=material_name)
    ET.SubElement(material, "color", rgba=" ".join(str(v) for v in color_rgba))
    return link


def add_revolute_joint(robot, name, parent, child, origin_xyz, origin_rpy,
                        axis_xyz, lower_rad, upper_rad,
                        effort=100.0, velocity=1.0):
    """Helper to add a revolute joint."""
    joint = ET.SubElement(robot, "joint", name=name, type="revolute")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin",
                  xyz=" ".join(str(v) for v in origin_xyz),
                  rpy=" ".join(str(v) for v in origin_rpy))
    ET.SubElement(joint, "axis", xyz=axis_xyz)
    ET.SubElement(joint, "limit",
                  lower=str(lower_rad),
                  upper=str(upper_rad),
                  effort=str(effort),
                  velocity=str(velocity))
    return joint


def add_fixed_joint(robot, name, parent, child, origin_xyz, origin_rpy=(0, 0, 0)):
    """Helper to add a fixed joint."""
    joint = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin",
                  xyz=" ".join(str(v) for v in origin_xyz),
                  rpy=" ".join(str(v) for v in origin_rpy))
    return joint


def create_urdf(joints_data, geometry_data, kinematics_data, mass_data):
    """
    Build URDF from design data.

    Kinematic chain per leg:
      pelvis
        └─ {side}_hip_roll_joint (revolute, X-axis)
             └─ {side}_hip_roll_link
                  └─ {side}_hip_pitch_joint (revolute, Y-axis)
                       └─ {side}_thigh
                            └─ {side}_knee_pitch_joint (revolute, Y-axis)
                                 └─ {side}_shin
                                      └─ {side}_ankle_pitch_joint (revolute, Y-axis) [MVS]
                                           └─ {side}_ankle
                                                └─ {side}_foot_joint (fixed)
                                                     └─ {side}_foot

    Torso chain:
      pelvis
        └─ torso_pitch_joint (revolute, Y-axis)
             └─ torso_pitch_link
                  └─ torso_roll_joint (revolute, X-axis)
                       └─ torso_roll_link
                            └─ torso_yaw_joint (revolute, Z-axis)
                                 └─ torso

    Axis directions are read from kinematics.yaml — not hardcoded.
    Joint limits are read from joints.yaml — not hardcoded.
    Mass and inertia are read from mass.yaml — not hardcoded.
    """

    robot = ET.Element("robot", name="megadroid_mvs")
    masses = mass_data["bodies"]

    # ── Geometry ──────────────────────────────────────────────────────────────
    geo = geometry_data["anthropometrics"]
    thigh_len  = geo["thigh_length_mm"]  / 1000.0
    shin_len   = geo["shin_length_mm"]   / 1000.0
    ankle_off  = geo["ankle_to_sole_offset_mm"] / 1000.0

    # Hip lateral offset: half the twin-rail inner face spacing
    hip_y_offset = geometry_data["leg_structure"]["twin_rail"]["inner_face_spacing_mm"] / 2000.0

    # ── Pelvis ────────────────────────────────────────────────────────────────
    pelvis_geom = ET.Element("box", size="0.20 0.30 0.15")
    pelvis_link = add_link(robot, "pelvis",
                           visual_origin_xyz=(0, 0, 0),
                           visual_origin_rpy=(0, 0, 0),
                           geometry=pelvis_geom,
                           color_rgba=(0.5, 0.5, 0.5, 1.0),
                           material_name="grey")
    ixx, iyy, izz = box_inertia(masses["pelvis"], 0.10, 0.15, 0.075)
    add_inertial(pelvis_link, masses["pelvis"], (0, 0, 0), ixx, iyy, izz)

    # ── Torso chain ───────────────────────────────────────────────────────────
    torso_pitch_geom = ET.Element("box", size="0.05 0.20 0.05")
    tp_link = add_link(robot, "torso_pitch_link",
                       visual_origin_xyz=(0, 0, 0),
                       visual_origin_rpy=(0, 0, 0),
                       geometry=torso_pitch_geom,
                       color_rgba=(0.5, 0.5, 0.5, 1.0),
                       material_name="grey")
    ixx, iyy, izz = box_inertia(masses["torso_pitch_link"], 0.025, 0.10, 0.025)
    add_inertial(tp_link, masses["torso_pitch_link"], (0, 0, 0), ixx, iyy, izz)

    lower, upper = get_limits(joints_data, "torso_pitch")
    add_revolute_joint(robot, "torso_pitch_joint",
                       parent="pelvis", child="torso_pitch_link",
                       origin_xyz=(0, 0, 0.075), origin_rpy=(0, 0, 0),
                       axis_xyz=get_joint_axis(kinematics_data, "torso_pitch"),
                       lower_rad=lower, upper_rad=upper)

    torso_roll_geom = ET.Element("box", size="0.05 0.20 0.05")
    tr_link = add_link(robot, "torso_roll_link",
                       visual_origin_xyz=(0, 0, 0),
                       visual_origin_rpy=(0, 0, 0),
                       geometry=torso_roll_geom,
                       color_rgba=(0.5, 0.5, 0.5, 1.0),
                       material_name="grey")
    ixx, iyy, izz = box_inertia(masses["torso_roll_link"], 0.025, 0.10, 0.025)
    add_inertial(tr_link, masses["torso_roll_link"], (0, 0, 0), ixx, iyy, izz)

    lower, upper = get_limits(joints_data, "torso_roll")
    add_revolute_joint(robot, "torso_roll_joint",
                       parent="torso_pitch_link", child="torso_roll_link",
                       origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0),
                       axis_xyz=get_joint_axis(kinematics_data, "torso_roll"),
                       lower_rad=lower, upper_rad=upper)

    torso_geom = ET.Element("box", size="0.15 0.25 0.40")
    torso_link = add_link(robot, "torso",
                          visual_origin_xyz=(0, 0, 0.20),
                          visual_origin_rpy=(0, 0, 0),
                          geometry=torso_geom,
                          color_rgba=(0.5, 0.5, 0.5, 1.0),
                          material_name="grey")
    ixx, iyy, izz = box_inertia(masses["torso"], 0.075, 0.125, 0.20)
    add_inertial(torso_link, masses["torso"], (0, 0, 0.20), ixx, iyy, izz)

    lower, upper = get_limits(joints_data, "torso_yaw")
    add_revolute_joint(robot, "torso_yaw_joint",
                       parent="torso_roll_link", child="torso",
                       origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0),
                       axis_xyz=get_joint_axis(kinematics_data, "torso_yaw"),
                       lower_rad=lower, upper_rad=upper)

    # ── Leg function ──────────────────────────────────────────────────────────
    def create_leg(side: str, y_sign: float):
        """
        Build one leg kinematic chain attached to the pelvis.
        y_sign: +1 for left leg, -1 for right leg.
        """
        p = side.lower()
        y = y_sign * hip_y_offset

        hip_roll_geom = ET.Element("sphere", radius="0.03")
        hr_link = add_link(robot, f"{p}_hip_roll_link",
                           visual_origin_xyz=(0, 0, 0),
                           visual_origin_rpy=(0, 0, 0),
                           geometry=hip_roll_geom,
                           color_rgba=(0.8, 0.4, 0.1, 1.0),
                           material_name="orange")
        i3 = sphere_inertia(masses["hip_roll_link"], 0.03)
        add_inertial(hr_link, masses["hip_roll_link"], (0, 0, 0), *i3)

        lower, upper = get_limits(joints_data, "hip_roll")
        add_revolute_joint(robot, f"{p}_hip_roll_joint",
                           parent="pelvis", child=f"{p}_hip_roll_link",
                           origin_xyz=(0, y, 0), origin_rpy=(0, 0, 0),
                           axis_xyz=get_joint_axis(kinematics_data, "hip_roll"),
                           lower_rad=lower, upper_rad=upper)

        thigh_geom = ET.Element("cylinder",
                                radius="0.040",
                                length=str(thigh_len))
        th_link = add_link(robot, f"{p}_thigh",
                           visual_origin_xyz=(0, 0, -thigh_len / 2),
                           visual_origin_rpy=(0, 0, 0),
                           geometry=thigh_geom,
                           color_rgba=(0.2, 0.2, 0.8, 1.0),
                           material_name="blue")
        ixx, iyy, izz = cylinder_inertia(masses["thigh"], 0.040, thigh_len / 2)
        add_inertial(th_link, masses["thigh"], (0, 0, -thigh_len / 2), ixx, iyy, izz)

        lower, upper = get_limits(joints_data, "hip_pitch")
        add_revolute_joint(robot, f"{p}_hip_pitch_joint",
                           parent=f"{p}_hip_roll_link", child=f"{p}_thigh",
                           origin_xyz=(0, 0, 0), origin_rpy=(0, 0, 0),
                           axis_xyz=get_joint_axis(kinematics_data, "hip_pitch"),
                           lower_rad=lower, upper_rad=upper)

        shin_geom = ET.Element("cylinder",
                               radius="0.035",
                               length=str(shin_len))
        sh_link = add_link(robot, f"{p}_shin",
                           visual_origin_xyz=(0, 0, -shin_len / 2),
                           visual_origin_rpy=(0, 0, 0),
                           geometry=shin_geom,
                           color_rgba=(0.2, 0.2, 0.8, 1.0),
                           material_name="blue")
        ixx, iyy, izz = cylinder_inertia(masses["shin"], 0.035, shin_len / 2)
        add_inertial(sh_link, masses["shin"], (0, 0, -shin_len / 2), ixx, iyy, izz)

        lower, upper = get_limits(joints_data, "knee_pitch")
        add_revolute_joint(robot, f"{p}_knee_pitch_joint",
                           parent=f"{p}_thigh", child=f"{p}_shin",
                           origin_xyz=(0, 0, -thigh_len), origin_rpy=(0, 0, 0),
                           axis_xyz=get_joint_axis(kinematics_data, "knee_pitch"),
                           lower_rad=lower, upper_rad=upper)

        if is_mvs_actuated(joints_data, "ankle_pitch"):
            ankle_geom = ET.Element("sphere", radius="0.025")
            an_link = add_link(robot, f"{p}_ankle",
                               visual_origin_xyz=(0, 0, 0),
                               visual_origin_rpy=(0, 0, 0),
                               geometry=ankle_geom,
                               color_rgba=(0.8, 0.4, 0.1, 1.0),
                               material_name="orange")
            i3 = sphere_inertia(masses["ankle"], 0.025)
            add_inertial(an_link, masses["ankle"], (0, 0, 0), *i3)

            lower, upper = get_limits(joints_data, "ankle_pitch")
            add_revolute_joint(robot, f"{p}_ankle_pitch_joint",
                               parent=f"{p}_shin", child=f"{p}_ankle",
                               origin_xyz=(0, 0, -shin_len), origin_rpy=(0, 0, 0),
                               axis_xyz=get_joint_axis(kinematics_data, "ankle_pitch"),
                               lower_rad=lower, upper_rad=upper)

            foot_parent = f"{p}_ankle"
            foot_origin_xyz = (0.05, 0, -ankle_off)
        else:
            foot_parent = f"{p}_shin"
            foot_origin_xyz = (0.05, 0, -(shin_len + ankle_off / 2))

        foot_geom = ET.Element("box", size="0.15 0.08 0.04")
        ft_link = add_link(robot, f"{p}_foot",
                           visual_origin_xyz=(0, 0, 0),
                           visual_origin_rpy=(0, 0, 0),
                           geometry=foot_geom,
                           color_rgba=(0.1, 0.1, 0.1, 1.0),
                           material_name="black")
        ixx, iyy, izz = box_inertia(masses["foot"], 0.075, 0.040, 0.020)
        add_inertial(ft_link, masses["foot"], (0, 0, 0), ixx, iyy, izz)

        add_fixed_joint(robot, f"{p}_foot_joint",
                        parent=foot_parent, child=f"{p}_foot",
                        origin_xyz=foot_origin_xyz)

    create_leg("left",  +1.0)
    create_leg("right", -1.0)

    return robot


def main():
    print("Generating URDF from authoritative design data...")
    print()

    joints_data     = load_yaml(DESIGN_DIR / "joints.yaml")
    geometry_data   = load_yaml(DESIGN_DIR / "geometry.yaml")
    kinematics_data = load_yaml(DESIGN_DIR / "kinematics.yaml")
    mass_data       = load_yaml(DESIGN_DIR / "mass.yaml")

    robot = create_urdf(joints_data, geometry_data, kinematics_data, mass_data)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    urdf_string = prettify_xml(robot)
    OUTPUT_FILE.write_text(urdf_string)

    print(f"✓ URDF generated: {OUTPUT_FILE}")
    print()
    print("Next steps:")
    print("  1. Verify dimensions: python3 tools/verify_urdf_dimensions.py")
    print("  2. Visualize robot:   python3 tools/visualize_urdf.py")
    print("  3. Generate MJCF:     python3 tools/generate_mjcf.py")


if __name__ == "__main__":
    main()
