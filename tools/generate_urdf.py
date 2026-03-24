#!/usr/bin/env python3
"""
Generate URDF from authoritative design YAML files.

Reads design/joints.yaml, design/geometry.yaml, and design/kinematics.yaml
and produces a URDF file suitable for visualization and kinematics analysis.

Output: simulation/urdf/megadroid_mvs.urdf
"""

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


def create_urdf(joints_data, geometry_data, kinematics_data):
    """
    Build URDF from design data.
    
    For MVS, we'll create a simplified kinematic chain:
    - Base link (pelvis)
    - Left leg: hip_roll -> hip_pitch -> knee_pitch -> foot
    - Right leg: hip_roll -> hip_pitch -> knee_pitch -> foot
    - Torso: pitch -> roll -> yaw -> torso_top
    
    Note: This is a kinematic skeleton for visualization, not a full
    mechanical model with collision geometry.
    """
    
    robot = ET.Element("robot", name="megadroid_mvs")
    
    # Extract geometry parameters
    geo = geometry_data["anthropometrics"]
    thigh_length = geo["thigh_length_mm"] / 1000.0  # Convert to meters
    shin_length = geo["shin_length_mm"] / 1000.0
    ankle_offset = geo["ankle_to_sole_offset_mm"] / 1000.0
    
    # Calculate pelvis height for standing pose with feet on ground
    # Pelvis should be at: thigh_length + shin_length + ankle_offset above ground
    pelvis_height = thigh_length + shin_length + ankle_offset
    
    # Extract joint data
    joints = joints_data["joints"]
    
    # Get nominal poses
    def get_nominal(joint_name):
        j = joints.get(joint_name, {})
        return j.get("nominal_stand_deg", 0.0)
    
    # =================================================================
    # BASE LINK (pelvis)
    # =================================================================
    base_link = ET.SubElement(robot, "link", name="pelvis")
    visual = ET.SubElement(base_link, "visual")
    origin_v = ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry_v = ET.SubElement(visual, "geometry")
    box_v = ET.SubElement(geometry_v, "box", size="0.2 0.3 0.15")
    material = ET.SubElement(visual, "material", name="grey")
    color = ET.SubElement(material, "color", rgba="0.5 0.5 0.5 1")
    
    # =================================================================
    # TORSO CHAIN (simplified single body for now)
    # =================================================================
    # For simplicity, we'll create a simple torso link
    # In a full model, you'd have torso_pitch, torso_roll, torso_yaw as separate joints
    
    torso_link = ET.SubElement(robot, "link", name="torso")
    visual_t = ET.SubElement(torso_link, "visual")
    origin_t = ET.SubElement(visual_t, "origin", xyz="0 0 0.3", rpy="0 0 0")
    geometry_t = ET.SubElement(visual_t, "geometry")
    box_t = ET.SubElement(geometry_t, "box", size="0.15 0.25 0.4")
    material_t = ET.SubElement(visual_t, "material", name="grey")
    color_t = ET.SubElement(material_t, "color", rgba="0.5 0.5 0.5 1")
    
    # Torso joint (fixed for MVS visualization - in reality has 3 DOF)
    torso_joint = ET.SubElement(robot, "joint", name="pelvis_to_torso", type="fixed")
    parent_t = ET.SubElement(torso_joint, "parent", link="pelvis")
    child_t = ET.SubElement(torso_joint, "child", link="torso")
    origin_tj = ET.SubElement(torso_joint, "origin", xyz="0 0 0.15", rpy="0 0 0")
    
    # =================================================================
    # LEG FUNCTION
    # =================================================================
    def create_leg(side, y_offset):
        """Create a leg (hip -> thigh -> knee -> shin -> foot)."""
        prefix = side.lower()
        
        # Hip joint (combining roll and pitch for simplicity)
        # In reality, hip_roll and hip_pitch are separate
        hip_link = ET.SubElement(robot, "link", name=f"{prefix}_hip")
        
        hip_joint = ET.SubElement(robot, "joint", name=f"{prefix}_hip_joint", type="revolute")
        parent_h = ET.SubElement(hip_joint, "parent", link="pelvis")
        child_h = ET.SubElement(hip_joint, "child", link=f"{prefix}_hip")
        origin_h = ET.SubElement(hip_joint, "origin", 
                                 xyz=f"0 {y_offset} 0", 
                                 rpy="0 0 0")
        axis_h = ET.SubElement(hip_joint, "axis", xyz="0 1 0")  # Pitch axis
        
        hip_limits = joints["hip_pitch"]["limits_deg"]
        limit_h = ET.SubElement(hip_joint, "limit",
                               lower=str(hip_limits["min"] * 3.14159 / 180),
                               upper=str(hip_limits["max"] * 3.14159 / 180),
                               effort="100",
                               velocity="1.0")
        
        # Thigh link
        thigh_link = ET.SubElement(robot, "link", name=f"{prefix}_thigh")
        visual_th = ET.SubElement(thigh_link, "visual")
        origin_th = ET.SubElement(visual_th, "origin", 
                                   xyz=f"0 0 {-thigh_length/2}", 
                                   rpy="0 0 0")
        geometry_th = ET.SubElement(visual_th, "geometry")
        cylinder_th = ET.SubElement(geometry_th, "cylinder", 
                                     radius="0.04", 
                                     length=str(thigh_length))
        material_th = ET.SubElement(visual_th, "material", name="blue")
        color_th = ET.SubElement(material_th, "color", rgba="0.2 0.2 0.8 1")
        
        # Thigh joint (connects hip to thigh)
        thigh_joint = ET.SubElement(robot, "joint", 
                                     name=f"{prefix}_thigh_joint", 
                                     type="fixed")
        parent_thj = ET.SubElement(thigh_joint, "parent", link=f"{prefix}_hip")
        child_thj = ET.SubElement(thigh_joint, "child", link=f"{prefix}_thigh")
        origin_thj = ET.SubElement(thigh_joint, "origin", xyz="0 0 0", rpy="0 0 0")
        
        # Knee link (just a joint, minimal geometry)
        knee_link = ET.SubElement(robot, "link", name=f"{prefix}_knee")
        
        knee_joint = ET.SubElement(robot, "joint", 
                                    name=f"{prefix}_knee_joint", 
                                    type="revolute")
        parent_k = ET.SubElement(knee_joint, "parent", link=f"{prefix}_thigh")
        child_k = ET.SubElement(knee_joint, "child", link=f"{prefix}_knee")
        origin_k = ET.SubElement(knee_joint, "origin", 
                                 xyz=f"0 0 {-thigh_length}", 
                                 rpy="0 0 0")
        axis_k = ET.SubElement(knee_joint, "axis", xyz="0 1 0")  # Pitch axis
        
        knee_limits = joints["knee_pitch"]["limits_deg"]
        limit_k = ET.SubElement(knee_joint, "limit",
                               lower=str(knee_limits["min"] * 3.14159 / 180),
                               upper=str(knee_limits["max"] * 3.14159 / 180),
                               effort="100",
                               velocity="1.0")
        
        # Shin link
        shin_link = ET.SubElement(robot, "link", name=f"{prefix}_shin")
        visual_sh = ET.SubElement(shin_link, "visual")
        origin_sh = ET.SubElement(visual_sh, "origin", 
                                   xyz=f"0 0 {-shin_length/2}", 
                                   rpy="0 0 0")
        geometry_sh = ET.SubElement(visual_sh, "geometry")
        cylinder_sh = ET.SubElement(geometry_sh, "cylinder", 
                                     radius="0.035", 
                                     length=str(shin_length))
        material_sh = ET.SubElement(visual_sh, "material", name="blue")
        color_sh = ET.SubElement(material_sh, "color", rgba="0.2 0.2 0.8 1")
        
        # Shin joint
        shin_joint = ET.SubElement(robot, "joint", 
                                    name=f"{prefix}_shin_joint", 
                                    type="fixed")
        parent_shj = ET.SubElement(shin_joint, "parent", link=f"{prefix}_knee")
        child_shj = ET.SubElement(shin_joint, "child", link=f"{prefix}_shin")
        origin_shj = ET.SubElement(shin_joint, "origin", xyz="0 0 0", rpy="0 0 0")
        
        # Foot link
        foot_link = ET.SubElement(robot, "link", name=f"{prefix}_foot")
        visual_f = ET.SubElement(foot_link, "visual")
        origin_f = ET.SubElement(visual_f, "origin", 
                                 xyz=f"0.05 0 {-ankle_offset/2}", 
                                 rpy="0 0 0")
        geometry_f = ET.SubElement(visual_f, "geometry")
        box_f = ET.SubElement(geometry_f, "box", size="0.15 0.08 0.04")
        material_f = ET.SubElement(visual_f, "material", name="black")
        color_f = ET.SubElement(material_f, "color", rgba="0.1 0.1 0.1 1")
        
        # Foot joint
        foot_joint = ET.SubElement(robot, "joint", 
                                    name=f"{prefix}_foot_joint", 
                                    type="fixed")
        parent_fj = ET.SubElement(foot_joint, "parent", link=f"{prefix}_shin")
        child_fj = ET.SubElement(foot_joint, "child", link=f"{prefix}_foot")
        # Foot is positioned shin_length + ankle_offset below shin origin
        origin_fj = ET.SubElement(foot_joint, "origin", 
                                  xyz=f"0 0 {-(shin_length + ankle_offset)}", 
                                  rpy="0 0 0")
    
    # Create both legs
    create_leg("left", 0.15)   # Left leg, offset +Y
    create_leg("right", -0.15)  # Right leg, offset -Y
    
    return robot


def main():
    print("Generating URDF from authoritative design data...")
    print()
    
    # Load design data
    joints_data = load_yaml(DESIGN_DIR / "joints.yaml")
    geometry_data = load_yaml(DESIGN_DIR / "geometry.yaml")
    kinematics_data = load_yaml(DESIGN_DIR / "kinematics.yaml")
    
    # Create URDF
    robot = create_urdf(joints_data, geometry_data, kinematics_data)
    
    # Ensure output directory exists
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    
    # Write URDF file
    urdf_string = prettify_xml(robot)
    OUTPUT_FILE.write_text(urdf_string)
    
    print(f"✓ URDF generated: {OUTPUT_FILE}")
    print()
    print("Next steps:")
    print("  1. Verify dimensions: python3 tools/verify_urdf_dimensions.py")
    print("  2. Visualize robot: python3 tools/visualize_urdf.py")


if __name__ == "__main__":
    main()
