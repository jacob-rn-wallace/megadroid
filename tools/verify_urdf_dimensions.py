#!/usr/bin/env python3
"""
Verify URDF dimensions match authoritative YAML geometry.

Checks that the generated URDF has correct link lengths and positions.
"""

import xml.etree.ElementTree as ET
import yaml
import numpy as np
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DESIGN_DIR = REPO_ROOT / "design"
URDF_PATH = REPO_ROOT / "simulation" / "urdf" / "megadroid_mvs.urdf"


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def parse_urdf_joints(urdf_path):
    """Extract joint transform data from URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    joints = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        origin = joint.find('origin')
        
        xyz = [0, 0, 0]
        if origin is not None and origin.get('xyz'):
            xyz = [float(x) for x in origin.get('xyz').split()]
        
        joints[name] = {
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'xyz': np.array(xyz)
        }
    
    return joints


def compute_leg_length(joints, side):
    """Compute total leg length from URDF joint transforms."""
    prefix = side.lower()

    # Hip pitch joint origin is relative to hip_roll_link (which sits at pelvis)
    # — its xyz offset is (0,0,0) so it contributes no vertical displacement.
    # Knee pitch joint origin is relative to thigh — z offset = thigh length.
    knee_joint_xyz  = joints.get(f'{prefix}_knee_pitch_joint',  {}).get('xyz', np.zeros(3))

    # Ankle pitch joint origin is relative to shin — z offset = shin length.
    ankle_joint_xyz = joints.get(f'{prefix}_ankle_pitch_joint', {}).get('xyz', np.zeros(3))

    # Foot joint origin is relative to ankle — z offset = ankle_to_sole_offset.
    foot_joint_xyz  = joints.get(f'{prefix}_foot_joint',        {}).get('xyz', np.zeros(3))

    thigh_length = abs(knee_joint_xyz[2])
    shin_length  = abs(ankle_joint_xyz[2])
    ankle_offset = abs(foot_joint_xyz[2])

    return {
        'thigh_length':  thigh_length,
        'shin_to_foot':  shin_length + ankle_offset,
        'total_length':  thigh_length + shin_length + ankle_offset,
    }


def main():
    print("=" * 60)
    print("URDF Dimension Verification")
    print("=" * 60)
    print()
    
    # Load authoritative geometry
    geometry = load_yaml(DESIGN_DIR / "geometry.yaml")
    geo = geometry["anthropometrics"]
    
    yaml_thigh = geo["thigh_length_mm"] / 1000.0
    yaml_shin = geo["shin_length_mm"] / 1000.0
    yaml_ankle = geo["ankle_to_sole_offset_mm"] / 1000.0
    yaml_total = yaml_thigh + yaml_shin + yaml_ankle
    
    print("YAML Geometry (authoritative):")
    print(f"  Thigh length:  {yaml_thigh:.3f} m ({geo['thigh_length_mm']} mm)")
    print(f"  Shin length:   {yaml_shin:.3f} m ({geo['shin_length_mm']} mm)")
    print(f"  Ankle offset:  {yaml_ankle:.3f} m ({geo['ankle_to_sole_offset_mm']} mm)")
    print(f"  Total leg:     {yaml_total:.3f} m")
    print()
    
    # Parse URDF
    if not URDF_PATH.exists():
        print(f"ERROR: URDF not found at {URDF_PATH}")
        print("Run 'python3 tools/generate_urdf.py' first")
        return
    
    joints = parse_urdf_joints(URDF_PATH)
    
    print("URDF Measurements:")
    print()
    
    for side in ['left', 'right']:
        leg_data = compute_leg_length(joints, side)
        
        print(f"{side.capitalize()} Leg:")
        print(f"  Thigh (hip→knee):      {leg_data['thigh_length']:.3f} m")
        print(f"  Shin+Ankle (knee→foot): {leg_data['shin_to_foot']:.3f} m")
        print(f"  Total leg length:      {leg_data['total_length']:.3f} m")
        print()
    
    # Check left leg against YAML
    left_leg = compute_leg_length(joints, 'left')
    
    print("=" * 60)
    print("Verification:")
    print("=" * 60)
    
    thigh_match = abs(left_leg['thigh_length'] - yaml_thigh) < 0.001
    shin_ankle_match = abs(left_leg['shin_to_foot'] - (yaml_shin + yaml_ankle)) < 0.001
    total_match = abs(left_leg['total_length'] - yaml_total) < 0.001
    
    print(f"Thigh length:       {'✓ MATCH' if thigh_match else '✗ MISMATCH'}")
    if not thigh_match:
        print(f"  Expected: {yaml_thigh:.3f} m")
        print(f"  Got:      {left_leg['thigh_length']:.3f} m")
    
    print(f"Shin+Ankle length:  {'✓ MATCH' if shin_ankle_match else '✗ MISMATCH'}")
    if not shin_ankle_match:
        print(f"  Expected: {yaml_shin + yaml_ankle:.3f} m")
        print(f"  Got:      {left_leg['shin_to_foot']:.3f} m")
    
    print(f"Total length:       {'✓ MATCH' if total_match else '✗ MISMATCH'}")
    if not total_match:
        print(f"  Expected: {yaml_total:.3f} m")
        print(f"  Got:      {left_leg['total_length']:.3f} m")
    
    print()
    
    if thigh_match and shin_ankle_match and total_match:
        print("✓ All dimensions match YAML geometry!")
    else:
        print("✗ URDF dimensions do not match YAML")
        print("   The URDF generator may have bugs.")


if __name__ == "__main__":
    main()
