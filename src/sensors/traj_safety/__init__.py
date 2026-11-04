"""Portable trajectory geometric safety (thick M2 core).

Sphere / hull table + self collision + near-miss. No Three.js.
See ``sensors/docs/traj-safety-sdk.md``.
"""

from __future__ import annotations

from .analyze import analyze_traj_safety, grade_from_counts
from .config import SafetyConfig
from .hull import analyze_hull_collisions
from .mesh import load_and_sample_stl, load_stl_vertices, sample_points
from .pipeline import analyze_joint_trajectory, analyze_joint_trajectory_dict
from .profile import SafetyProfile, list_safety_profile_ids, load_safety_profile
from .sphere import analyze_sphere_collisions
from .types import SCHEMA_INPUT, SCHEMA_RESULT

__all__ = [
    "SCHEMA_INPUT",
    "SCHEMA_RESULT",
    "SafetyConfig",
    "SafetyProfile",
    "analyze_hull_collisions",
    "analyze_joint_trajectory",
    "analyze_joint_trajectory_dict",
    "analyze_sphere_collisions",
    "analyze_traj_safety",
    "grade_from_counts",
    "list_safety_profile_ids",
    "load_and_sample_stl",
    "load_safety_profile",
    "load_stl_vertices",
    "sample_points",
]
