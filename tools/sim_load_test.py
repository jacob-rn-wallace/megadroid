#!/usr/bin/env python3
"""
MuJoCo load test — quick sanity check for the simulation model.

Verifies the MJCF loads without errors, reports model statistics,
and runs a single physics step to confirm the model is simulation-ready.

Usage:
    python3 tools/sim_load_test.py
"""

import mujoco
import numpy as np
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MJCF_PATH = REPO_ROOT / "simulation" / "mujoco" / "megadroid_mvs.xml"


def main():
    print("=" * 60)
    print("MuJoCo Load Test")
    print("=" * 60)
    print()

    if not MJCF_PATH.exists():
        print(f"ERROR: MJCF not found at {MJCF_PATH}")
        print("Run: python3 tools/generate_mjcf.py")
        raise SystemExit(1)

    print(f"Loading: {MJCF_PATH.relative_to(REPO_ROOT)}")
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data  = mujoco.MjData(model)
    print("✓ Model loaded")
    print()

    print("Model statistics:")
    print(f"  Bodies:     {model.nbody}  (includes world)")
    print(f"  DOF (nv):   {model.nv}  (11 joints + 6 freejoint)")
    print(f"  Actuators:  {model.nu}")
    print(f"  Geoms:      {model.ngeom}")
    total_mass = sum(model.body_mass)
    print(f"  Total mass: {total_mass:.2f} kg")
    print()

    print("Bodies and masses:")
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        mass = model.body_mass[i]
        print(f"  [{i:2d}] {name or 'world':<28s}  {mass:.3f} kg")
    print()

    print("Actuators:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  [{i:2d}] {name}")
    print()

    # Single physics step
    mujoco.mj_resetData(model, data)
    # Position pelvis at correct height for initial standing pose
    root_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    root_adr = model.jnt_qposadr[root_id]
    data.qpos[root_adr + 2] = sum(model.body_mass[1:]) / sum(model.body_mass[1:])  # height set in XML
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    print("✓ Physics step succeeded — model is simulation-ready")
    print()

    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    print(f"  Pelvis position after 1 step: "
          f"({data.xpos[pelvis_id][0]:.3f}, "
          f"{data.xpos[pelvis_id][1]:.3f}, "
          f"{data.xpos[pelvis_id][2]:.3f}) m")


if __name__ == "__main__":
    main()
