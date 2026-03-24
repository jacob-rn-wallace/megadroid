#!/usr/bin/env python3
"""
Visualize Megadroid URDF using matplotlib (macOS-compatible).

Loads the generated URDF and provides a simple 3D visualization.

Requirements:
    pip install matplotlib numpy lxml

Usage:
    python3 tools/visualize_urdf.py
"""

import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import math

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = REPO_ROOT / "simulation" / "urdf" / "megadroid_mvs.urdf"


def parse_urdf(urdf_path):
    """Parse URDF and extract joint/link structure."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    joints = {}
    links = {}
    
    for joint in root.findall('joint'):
        name = joint.get('name')
        joint_type = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        
        origin = joint.find('origin')
        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if origin is not None:
            if origin.get('xyz'):
                xyz = [float(x) for x in origin.get('xyz').split()]
            if origin.get('rpy'):
                rpy = [float(x) for x in origin.get('rpy').split()]
        
        joints[name] = {
            'type': joint_type,
            'parent': parent,
            'child': child,
            'xyz': xyz,
            'rpy': rpy
        }
    
    for link in root.findall('link'):
        name = link.get('name')
        links[name] = {'name': name}
    
    return joints, links


def compute_link_positions(joints, base_link='pelvis'):
    """Compute 3D positions of all links in the kinematic tree."""
    positions = {base_link: np.array([0, 0, 0])}
    
    def process_children(parent_link, parent_pos):
        for joint_name, joint_data in joints.items():
            if joint_data['parent'] == parent_link:
                child_link = joint_data['child']
                # Transform from parent
                offset = np.array(joint_data['xyz'])
                child_pos = parent_pos + offset
                positions[child_link] = child_pos
                # Recursively process children
                process_children(child_link, child_pos)
    
    process_children(base_link, positions[base_link])
    return positions


def visualize_robot(urdf_path):
    """Visualize the robot skeleton in 3D."""
    joints, links = parse_urdf(urdf_path)
    positions = compute_link_positions(joints)
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot links as points
    for link_name, pos in positions.items():
        ax.scatter(*pos, s=100, c='blue', marker='o')
        ax.text(pos[0], pos[1], pos[2], f'  {link_name}', fontsize=8)
    
    # Plot joints as connections
    for joint_name, joint_data in joints.items():
        parent = joint_data['parent']
        child = joint_data['child']
        
        if parent in positions and child in positions:
            p1 = positions[parent]
            p2 = positions[child]
            
            color = 'green' if joint_data['type'] == 'revolute' else 'gray'
            linewidth = 3 if joint_data['type'] == 'revolute' else 1
            
            ax.plot([p1[0], p2[0]], 
                   [p1[1], p2[1]], 
                   [p1[2], p2[2]], 
                   color=color, linewidth=linewidth)
    
    # Ground plane (positioned below robot)
    # Find minimum Z coordinate to place ground appropriately
    min_z = min(pos[2] for pos in positions.values())
    ground_level = min_z - 0.05  # Slightly below lowest point
    
    xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 10), 
                         np.linspace(-0.5, 0.5, 10))
    zz = np.full_like(xx, ground_level)
    ax.plot_surface(xx, yy, zz, alpha=0.2, color='tan')
    
    # Labels and formatting
    ax.set_xlabel('X (forward) [m]')
    ax.set_ylabel('Y (lateral) [m]')
    ax.set_zlabel('Z (vertical) [m]')
    
    # Calculate and display actual leg segment lengths
    if 'left_hip' in positions and 'left_knee' in positions and 'left_foot' in positions:
        thigh_vec = positions['left_knee'] - positions['left_hip']
        thigh_length = np.linalg.norm(thigh_vec)
        
        shin_vec = positions['left_foot'] - positions['left_knee']
        shin_length = np.linalg.norm(shin_vec)
        
        title = (f'Megadroid MVS - Kinematic Structure\n'
                f'(Green = Revolute joints, Gray = Fixed)\n'
                f'Left leg: Thigh={thigh_length:.3f}m, Shin+Ankle={shin_length:.3f}m')
        ax.set_title(title)
    else:
        ax.set_title('Megadroid MVS - Kinematic Structure\n(Green = Revolute joints, Gray = Fixed)')
    
    # Set equal aspect ratio to reduce visual distortion
    max_range = 0.7
    mid_x = 0
    mid_y = 0
    mid_z = max_range / 2
    
    ax.set_xlim([mid_x - max_range/2, mid_x + max_range/2])
    ax.set_ylim([mid_y - max_range/2, mid_y + max_range/2])
    ax.set_zlim([0, max_range])
    
    # Force equal aspect ratio (helps reduce distortion)
    ax.set_box_aspect([1, 1, 1])
    
    # Better viewing angle
    ax.view_init(elev=20, azim=45)
    
    plt.tight_layout()
    plt.show()


def main():
    if not URDF_PATH.exists():
        print(f"ERROR: URDF not found at {URDF_PATH}")
        print("Run 'python3 tools/generate_urdf.py' first")
        return
    
    print("Visualizing Megadroid URDF...")
    print(f"Loading: {URDF_PATH}")
    print()
    print("Controls:")
    print("  - Click and drag to rotate")
    print("  - Scroll to zoom")
    print("  - Close window to exit")
    print()
    
    visualize_robot(URDF_PATH)
    
    print("✓ Visualization complete")


if __name__ == "__main__":
    main()
