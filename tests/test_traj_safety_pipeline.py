"""Phase B: FK link poses + safety profile pipeline."""

from __future__ import annotations

import math

from sensors.cs66_kinematics import fk_link_poses_from_joints_rad
from sensors.traj_safety import (
    analyze_joint_trajectory,
    load_safety_profile,
)


def test_fk_link_poses_count() -> None:
    # teach pose #1 joints (deg) → rad
    q_deg = [46.889, -62.737, 120.028, -150.651, 90.667, 40.356]
    q = [math.radians(x) for x in q_deg]
    poses = fk_link_poses_from_joints_rad(q, profile_id="elite_cs66")
    assert len(poses) == 7
    flange = poses[-1]
    # teach position_mm ≈ [-232.958, -463.942, 313.466]
    xyz_mm = flange[:3, 3] * 1000.0
    assert abs(float(xyz_mm[0]) + 232.958) < 0.5
    assert abs(float(xyz_mm[1]) + 463.942) < 0.5
    assert abs(float(xyz_mm[2]) - 313.466) < 0.5


def test_load_elite_cs66_safety_profile() -> None:
    prof = load_safety_profile("elite_cs66")
    assert prof.id == "elite_cs66"
    assert prof.kinematics_profile_id == "elite_cs66"
    assert len(prof.links) >= 7
    assert len(prof.capsules) >= 1


def test_analyze_joint_trajectory_runs() -> None:
    q_deg = [46.889, -62.737, 120.028, -150.651, 90.667, 40.356]
    q = [math.radians(x) for x in q_deg]
    # static pose repeated — should be geometrically stable (no huge self-hit required)
    series = [q, q, q]
    out = analyze_joint_trajectory(series, profile="elite_cs66", stride=1)
    assert out["ok"] is True
    assert out["schema"] == "sensors_traj_safety_v1"
    assert out["n_frames"] == 3
    assert out["grade"] in {"good", "warn", "bad"}
    assert "n_table" in out and "n_self" in out
