#!/usr/bin/env python3
"""
Analyze joint workspace and validate ranges for Megadroid MVS.

Computes forward kinematics across joint ranges to:
- Verify joints can reach their limits without issues
- Map foot workspace (reachable positions)
- Check for singularities or problematic configurations

Requirements:
    pip install numpy matplotlib pyyaml

Usage:
    python3 tools/analyze_workspace.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import yaml
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DESIGN_DIR = REPO_ROOT / "design"


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def rotation_matrix_y(theta):
    """Rotation matrix around Y axis (pitch)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


def forward_kinematics_leg(hip_pitch, knee_pitch, thigh_len, shin_len, ankle_offset, hip_height):
    """
    Compute foot position given joint angles.
    
    Simplified FK for one leg:
    - Hip at (0, 0, hip_height) - elevated above ground
    - Hip pitch rotates thigh
    - Knee pitch rotates shin
    - Returns foot position in 3D (absolute coordinates, ground at Z=0)
    
    Args:
        hip_pitch: Hip pitch angle (radians)
        knee_pitch: Knee pitch angle (radians)
        thigh_len: Thigh length (meters)
        shin_len: Shin length (meters)
        ankle_offset: Ankle to sole offset (meters)
        hip_height: Hip height above ground (meters)
    
    Returns:
        foot_pos: 3D position of foot [x, y, z]
    """
    # Start at hip (elevated above ground)
    pos = np.array([0, 0, hip_height])
    
    # Hip pitch: thigh rotates around Y axis
    R_hip = rotation_matrix_y(hip_pitch)
    thigh_vec = R_hip @ np.array([0, 0, -thigh_len])
    knee_pos = pos + thigh_vec
    
    # Knee pitch: shin rotates around Y axis, but relative to thigh orientation
    R_knee = rotation_matrix_y(hip_pitch + knee_pitch)
    shin_vec = R_knee @ np.array([0, 0, -(shin_len + ankle_offset)])
    foot_pos = knee_pos + shin_vec
    
    return foot_pos


def sample_workspace(joints, geometry):
    """
    Sample the workspace by varying joint angles across their ranges.
    
    Returns array of reachable foot positions.
    """
    # Extract parameters
    thigh_len = geometry["anthropometrics"]["thigh_length_mm"] / 1000.0
    shin_len = geometry["anthropometrics"]["shin_length_mm"] / 1000.0
    ankle_offset = geometry["anthropometrics"]["ankle_to_sole_offset_mm"] / 1000.0
    hip_height = thigh_len + shin_len + ankle_offset  # Hip height in nominal straight-leg stand
    
    hip_pitch_limits = joints["hip_pitch"]["limits_deg"]
    knee_pitch_limits = joints["knee_pitch"]["limits_deg"]
    
    # Convert to radians
    hip_min = np.deg2rad(hip_pitch_limits["min"])
    hip_max = np.deg2rad(hip_pitch_limits["max"])
    knee_min = np.deg2rad(knee_pitch_limits["min"])
    knee_max = np.deg2rad(knee_pitch_limits["max"])
    
    # Sample across ranges
    n_samples = 20
    hip_angles = np.linspace(hip_min, hip_max, n_samples)
    knee_angles = np.linspace(knee_min, knee_max, n_samples)
    
    workspace = []
    
    for hip in hip_angles:
        for knee in knee_angles:
            foot_pos = forward_kinematics_leg(hip, knee, thigh_len, shin_len, ankle_offset, hip_height)
            workspace.append(foot_pos)
    
    return np.array(workspace)


def check_nominal_pose(joints, geometry):
    """Check the nominal standing pose is valid."""
    thigh_len = geometry["anthropometrics"]["thigh_length_mm"] / 1000.0
    shin_len = geometry["anthropometrics"]["shin_length_mm"] / 1000.0
    ankle_offset = geometry["anthropometrics"]["ankle_to_sole_offset_mm"] / 1000.0
    hip_height = thigh_len + shin_len + ankle_offset
    
    hip_nominal = np.deg2rad(joints["hip_pitch"].get("nominal_stand_deg", 0))
    knee_nominal = np.deg2rad(joints["knee_pitch"].get("nominal_stand_deg", 0))
    
    foot_pos = forward_kinematics_leg(hip_nominal, knee_nominal, 
                                       thigh_len, shin_len, ankle_offset, hip_height)
    
    return foot_pos


def analyze_workspace():
    """Main analysis function."""
    print("=" * 60)
    print("Megadroid MVS — Workspace Analysis")
    print("=" * 60)
    print()
    
    # Load design data
    joints = load_yaml(DESIGN_DIR / "joints.yaml")["joints"]
    geometry = load_yaml(DESIGN_DIR / "geometry.yaml")
    
    # Extract parameters
    thigh_len = geometry["anthropometrics"]["thigh_length_mm"] / 1000.0
    shin_len = geometry["anthropometrics"]["shin_length_mm"] / 1000.0
    ankle_offset = geometry["anthropometrics"]["ankle_to_sole_offset_mm"] / 1000.0
    total_leg = thigh_len + shin_len + ankle_offset
    
    print("Leg Geometry:")
    print(f"  Thigh length:      {thigh_len:.3f} m")
    print(f"  Shin length:       {shin_len:.3f} m")
    print(f"  Ankle offset:      {ankle_offset:.3f} m")
    print(f"  Total leg length:  {total_leg:.3f} m")
    print()
    
    # Joint limits
    hip_limits = joints["hip_pitch"]["limits_deg"]
    knee_limits = joints["knee_pitch"]["limits_deg"]
    
    print("Joint Limits:")
    print(f"  Hip pitch:   [{hip_limits['min']:>4}°, {hip_limits['max']:>4}°]")
    print(f"  Knee pitch:  [{knee_limits['min']:>4}°, {knee_limits['max']:>4}°]")
    print()
    
    # Check nominal pose
    print("Nominal Standing Pose:")
    nominal_foot = check_nominal_pose(joints, geometry)
    print(f"  Hip pitch:   {joints['hip_pitch'].get('nominal_stand_deg', 0)}°")
    print(f"  Knee pitch:  {joints['knee_pitch'].get('nominal_stand_deg', 0)}°")
    print(f"  Foot position: [{nominal_foot[0]:.3f}, {nominal_foot[1]:.3f}, {nominal_foot[2]:.3f}] m")
    print(f"  Foot Z (height): {nominal_foot[2]:.3f} m")
    
    if abs(nominal_foot[2]) > 0.01:
        print(f"  ⚠ WARNING: Foot not on ground (Z should be ~0)")
    else:
        print(f"  ✓ Foot is on ground")
    print()
    
    # Sample workspace
    print("Sampling workspace...")
    workspace = sample_workspace(joints, geometry)
    
    print(f"✓ Sampled {len(workspace)} configurations")
    print()
    
    # Analyze workspace
    print("Workspace Analysis:")
    print(f"  X range: [{workspace[:, 0].min():.3f}, {workspace[:, 0].max():.3f}] m")
    print(f"  Y range: [{workspace[:, 1].min():.3f}, {workspace[:, 1].max():.3f}] m")
    print(f"  Z range: [{workspace[:, 2].min():.3f}, {workspace[:, 2].max():.3f}] m")
    
    forward_reach = workspace[:, 0].max()
    backward_reach = workspace[:, 0].min()
    upward_reach = workspace[:, 2].max()
    downward_reach = workspace[:, 2].min()
    
    print()
    print("Reachability:")
    print(f"  Forward reach:   {forward_reach:.3f} m")
    print(f"  Backward reach:  {backward_reach:.3f} m")
    print(f"  Upward reach:    {upward_reach:.3f} m")
    print(f"  Downward reach:  {downward_reach:.3f} m")
    print()
    
    # Check if workspace includes ground level
    if downward_reach > 0.01:
        print("  ⚠ WARNING: Foot cannot reach ground (lowest Z > 0)")
    else:
        print("  ✓ Foot can reach ground level")
    
    # Visualize
    visualize_workspace(workspace, nominal_foot, thigh_len, shin_len, ankle_offset)


def visualize_workspace(workspace, nominal_foot, thigh_len, shin_len, ankle_offset):
    """Create 3D visualization of workspace."""
    fig = plt.figure(figsize=(14, 6))
    
    # 3D workspace plot
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(workspace[:, 0], workspace[:, 1], workspace[:, 2], 
               c=workspace[:, 2], cmap='viridis', s=20, alpha=0.6)
    ax1.scatter(*nominal_foot, c='red', s=200, marker='*', 
               label='Nominal pose', edgecolors='black', linewidths=2)
    
    # Ground plane
    xx, yy = np.meshgrid(np.linspace(-0.3, 0.3, 10), 
                         np.linspace(-0.2, 0.2, 10))
    zz = np.zeros_like(xx)
    ax1.plot_surface(xx, yy, zz, alpha=0.2, color='tan')
    
    ax1.set_xlabel('X (forward) [m]')
    ax1.set_ylabel('Y (lateral) [m]')
    ax1.set_zlabel('Z (vertical) [m]')
    ax1.set_title('Foot Workspace (3D)')
    ax1.legend()
    ax1.set_box_aspect([1, 1, 1])
    
    # Side view (X-Z plane)
    ax2 = fig.add_subplot(122)
    ax2.scatter(workspace[:, 0], workspace[:, 2], 
               c=workspace[:, 2], cmap='viridis', s=20, alpha=0.6)
    ax2.scatter(nominal_foot[0], nominal_foot[2], c='red', s=200, marker='*',
               label='Nominal pose', edgecolors='black', linewidths=2)
    ax2.axhline(0, color='tan', linewidth=2, label='Ground')
    ax2.set_xlabel('X (forward) [m]')
    ax2.set_ylabel('Z (vertical) [m]')
    ax2.set_title('Foot Workspace (Side View)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    plt.tight_layout()
    plt.show()


def main():
    analyze_workspace()


if __name__ == "__main__":
    main()
