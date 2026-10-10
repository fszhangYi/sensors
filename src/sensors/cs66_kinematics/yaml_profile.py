from __future__ import annotations

"""Load arm kinematics profiles from YAML (no hardcoded robot geometry).

Profile files live under ``cs66_kinematics/profiles/*.yaml``.
Select via ``profile_id`` argument or env ``CS66_KINEMATICS_PROFILE``
(default: ``elite_ec616`` for parity with legacy ``sensors.kinematics``).
"""

import os
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Any, Literal, Mapping, Optional, Tuple

import numpy as np
import yaml

from .model import DhRow, TeachPendantPose

DhConvention = Literal["link_lengths_standard", "explicit_dh_table", "mdh"]

DEFAULT_PROFILE_ID = "elite_ec616"
ENV_PROFILE_ID = "CS66_KINEMATICS_PROFILE"

PROFILES_DIR = Path(__file__).resolve().parent / "profiles"


@dataclass(frozen=True)
class ArmYamlProfile:
    """Validated kinematics parameters loaded from one YAML file."""

    id: str
    brand: str
    product: str
    supported: bool
    notes: str
    manipulator_type: int
    dh_convention: DhConvention
    link_length_mm: Tuple[float, float, float, float, float, float]
    joint_offset_deg: Tuple[float, float, float, float, float, float]
    joint_direction: Tuple[float, float, float, float, float, float]
    default_joint_angle_mode: str
    base_cs_x: int
    base_cs_z: int
    install_note: str
    install_rot_z1_deg: float
    install_rot_xb_q1_deg: float
    default_tcp_xyz_m: Tuple[float, float, float]
    soft_min_deg: Tuple[float, float, float, float, float, float]
    soft_max_deg: Tuple[float, float, float, float, float, float]
    joint_vel_max_deg_s: Tuple[float, float, float, float, float, float]
    joint_vel_min_deg_s: Tuple[float, float, float, float, float, float]
    joint_accel_max_deg_s2: Tuple[float, float, float, float, float, float]
    cart_vel_max_mm_s: float
    cart_vel_min_mm_s: float
    cart_accel_max_mm_s2: float
    ori_vel_max_deg_s: float
    ori_vel_min_deg_s: float
    ori_accel_max_deg_s2: float
    teach_poses: Tuple[TeachPendantPose, ...] = field(default_factory=tuple)
    # Optional geometry overrides
    mdh_alpha_rad: Optional[Tuple[float, ...]] = None
    mdh_a_m: Optional[Tuple[float, ...]] = None
    mdh_d_m: Optional[Tuple[float, ...]] = None
    dh_table: Optional[Tuple[DhRow, ...]] = None
    source_path: Optional[str] = None


def profiles_dir() -> Path:
    return PROFILES_DIR


def list_profile_ids() -> list[str]:
    return sorted(p.stem for p in PROFILES_DIR.glob("*.yaml") if p.is_file())


def resolve_profile_id(profile_id: Optional[str] = None) -> str:
    if profile_id is not None and str(profile_id).strip():
        return str(profile_id).strip()
    env = os.environ.get(ENV_PROFILE_ID, "").strip()
    if env:
        return env
    return DEFAULT_PROFILE_ID


def _require_len6(name: str, raw: Any) -> Tuple[float, float, float, float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 6:
        raise ValueError(f"{name} must be a list of 6 numbers")
    out = tuple(float(x) for x in raw)
    if any(not np.isfinite(x) for x in out):
        raise ValueError(f"{name} must be finite")
    return out  # type: ignore[return-value]


def _require_len3(name: str, raw: Any) -> Tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError(f"{name} must be a list of 3 numbers")
    out = tuple(float(x) for x in raw)
    if any(not np.isfinite(x) for x in out):
        raise ValueError(f"{name} must be finite")
    return out  # type: ignore[return-value]


def _parse_teach_poses(raw: Any) -> Tuple[TeachPendantPose, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ValueError("teach_poses must be a list")
    poses: list[TeachPendantPose] = []
    for i, item in enumerate(raw):
        if not isinstance(item, Mapping):
            raise ValueError(f"teach_poses[{i}] must be a mapping")
        idx = int(item.get("index", i + 1))
        jd = _require_len6(f"teach_poses[{i}].joint_deg", item.get("joint_deg"))
        pos = _require_len3(f"teach_poses[{i}].position_mm", item.get("position_mm"))
        ori = _require_len3(
            f"teach_poses[{i}].orientation_rxyz_deg", item.get("orientation_rxyz_deg")
        )
        poses.append(TeachPendantPose(idx, jd, pos, ori))
    return tuple(poses)


def _parse_dh_table(raw: Any) -> Tuple[DhRow, ...]:
    if not isinstance(raw, list) or len(raw) != 6:
        raise ValueError("dh_table must be a list of 6 rows")
    rows: list[DhRow] = []
    for i, row in enumerate(raw):
        if isinstance(row, Mapping):
            th = float(row["theta_offset"])
            d = float(row["d"])
            a = float(row["a"])
            al = float(row["alpha"])
        elif isinstance(row, (list, tuple)) and len(row) == 4:
            th, d, a, al = (float(x) for x in row)
        else:
            raise ValueError(f"dh_table[{i}] must be mapping or 4-list")
        rows.append((th, d, a, al))
    return tuple(rows)


def _parse_mdh(raw: Any) -> Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]]:
    if not isinstance(raw, Mapping):
        raise ValueError("mdh must be a mapping with alpha_rad, a_m, d_m")
    alpha = _require_len6("mdh.alpha_rad", raw.get("alpha_rad"))
    a = _require_len6("mdh.a_m", raw.get("a_m"))
    d = _require_len6("mdh.d_m", raw.get("d_m"))
    return alpha, a, d


def profile_from_mapping(data: Mapping[str, Any], *, source_path: Optional[str] = None) -> ArmYamlProfile:
    """Parse and validate a profile dict (as loaded from YAML)."""
    pid = str(data.get("id") or "").strip()
    if not pid:
        raise ValueError("profile id is required")

    conv = str(data.get("dh_convention") or "link_lengths_standard").strip()
    if conv not in ("link_lengths_standard", "explicit_dh_table", "mdh"):
        raise ValueError(f"unsupported dh_convention: {conv!r}")

    install = data.get("install") or {}
    if not isinstance(install, Mapping):
        raise ValueError("install must be a mapping")
    soft = data.get("soft_limits_deg") or {}
    if not isinstance(soft, Mapping):
        raise ValueError("soft_limits_deg must be a mapping")
    cart = data.get("cartesian") or {}
    if not isinstance(cart, Mapping):
        raise ValueError("cartesian must be a mapping")

    mdh_alpha = mdh_a = mdh_d = None
    if data.get("mdh") is not None:
        mdh_alpha, mdh_a, mdh_d = _parse_mdh(data["mdh"])

    dh_table = None
    if data.get("dh_table") is not None:
        dh_table = _parse_dh_table(data["dh_table"])

    return ArmYamlProfile(
        id=pid,
        brand=str(data.get("brand") or ""),
        product=str(data.get("product") or ""),
        supported=bool(data.get("supported", True)),
        notes=str(data.get("notes") or ""),
        manipulator_type=int(data.get("manipulator_type", 1)),
        dh_convention=conv,  # type: ignore[arg-type]
        link_length_mm=_require_len6("link_length_mm", data.get("link_length_mm")),
        joint_offset_deg=_require_len6("joint_offset_deg", data.get("joint_offset_deg")),
        joint_direction=_require_len6("joint_direction", data.get("joint_direction")),
        default_joint_angle_mode=str(data.get("default_joint_angle_mode") or "machine_deg"),
        base_cs_x=int(data.get("base_cs_x", 6)),
        base_cs_z=int(data.get("base_cs_z", 1)),
        install_note=str(install.get("note") or ""),
        install_rot_z1_deg=float(install.get("rot_z1_deg", 0.0)),
        install_rot_xb_q1_deg=float(install.get("rot_xb_q1_deg", 0.0)),
        default_tcp_xyz_m=_require_len3("default_tcp_xyz_m", data.get("default_tcp_xyz_m", [0, 0, 0])),
        soft_min_deg=_require_len6("soft_limits_deg.min", soft.get("min")),
        soft_max_deg=_require_len6("soft_limits_deg.max", soft.get("max")),
        joint_vel_max_deg_s=_require_len6("joint_vel_max_deg_s", data.get("joint_vel_max_deg_s")),
        joint_vel_min_deg_s=_require_len6("joint_vel_min_deg_s", data.get("joint_vel_min_deg_s")),
        joint_accel_max_deg_s2=_require_len6(
            "joint_accel_max_deg_s2", data.get("joint_accel_max_deg_s2")
        ),
        cart_vel_max_mm_s=float(cart.get("vel_max_mm_s", 1500.0)),
        cart_vel_min_mm_s=float(cart.get("vel_min_mm_s", 1.0)),
        cart_accel_max_mm_s2=float(cart.get("accel_max_mm_s2", 9000.0)),
        ori_vel_max_deg_s=float(cart.get("ori_vel_max_deg_s", 150.0)),
        ori_vel_min_deg_s=float(cart.get("ori_vel_min_deg_s", 5.0)),
        ori_accel_max_deg_s2=float(cart.get("ori_accel_max_deg_s2", 400.0)),
        teach_poses=_parse_teach_poses(data.get("teach_poses")),
        mdh_alpha_rad=mdh_alpha,
        mdh_a_m=mdh_a,
        mdh_d_m=mdh_d,
        dh_table=dh_table,
        source_path=source_path,
    )


def load_profile_file(path: Path | str) -> ArmYamlProfile:
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, Mapping):
        raise ValueError(f"YAML root must be a mapping: {p}")
    return profile_from_mapping(data, source_path=str(p))


@lru_cache(maxsize=32)
def load_profile(profile_id: str) -> ArmYamlProfile:
    """Load ``profiles/{profile_id}.yaml`` (cached)."""
    pid = str(profile_id).strip()
    path = PROFILES_DIR / f"{pid}.yaml"
    if not path.is_file():
        known = ", ".join(list_profile_ids()) or "(none)"
        raise FileNotFoundError(f"kinematics profile not found: {pid!r} (known: {known})")
    return load_profile_file(path)


def clear_profile_cache() -> None:
    load_profile.cache_clear()


def get_profile(profile_id: Optional[str] = None) -> ArmYamlProfile:
    return load_profile(resolve_profile_id(profile_id))


def yaml_migration_checklist(profile: ArmYamlProfile) -> dict[str, Any]:
    """Report whether this profile can run FK/IK with YAML data alone (same solver)."""
    issues: list[str] = []
    if not profile.supported:
        issues.append("supported is false — refuse build until calibrated")
    if profile.manipulator_type != 1:
        issues.append(
            f"manipulator_type={profile.manipulator_type} not supported by fk/ik "
            "(only type 1 / 6-DOF offset-wrist)"
        )
    if profile.dh_convention == "link_lengths_standard":
        if all(abs(x) < 1e-15 for x in profile.link_length_mm):
            issues.append("link_length_mm is all zeros — fill geometry")
    elif profile.dh_convention == "explicit_dh_table":
        if profile.dh_table is None:
            issues.append("dh_convention=explicit_dh_table but dh_table missing")
    elif profile.dh_convention == "mdh":
        if profile.mdh_alpha_rad is None:
            issues.append("dh_convention=mdh but mdh block missing")
    if not profile.teach_poses:
        issues.append("teach_poses empty — FK regression cannot be validated from YAML alone")

    code_locked = [
        "6-DOF revolute chain solver (fk_standard_dh / numerical ik_flange)",
        "link_lengths_standard assumes fixed offset-wrist alpha pattern in dh.py",
        "joint_angle_mode semantics (machine_deg / pendant_deg / …) are code paths",
        "base axis enum → vector map (_AXIS_VEC) is shared code, not per-YAML",
        "non type-1 manipulators / different joint count need code changes",
    ]
    return {
        "profile_id": profile.id,
        "yaml_only_same_topology": len(issues) == 0 and profile.manipulator_type == 1,
        "blocking_issues": issues,
        "code_locked_assumptions": code_locked,
        "source_path": profile.source_path,
    }
