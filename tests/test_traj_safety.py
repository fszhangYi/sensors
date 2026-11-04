"""v0 traj_safety: sphere/hull + near-miss + grade."""

from __future__ import annotations

from sensors.traj_safety import (
    SafetyConfig,
    analyze_hull_collisions,
    analyze_sphere_collisions,
    analyze_traj_safety,
    grade_from_counts,
)


def test_grade_from_counts() -> None:
    assert grade_from_counts(n_table=0, n_self=0, n_near_miss=0) == "good"
    assert grade_from_counts(n_table=0, n_self=0, n_near_miss=2) == "warn"
    assert grade_from_counts(n_table=1, n_self=0, n_near_miss=0) == "bad"
    assert grade_from_counts(n_table=0, n_self=1, n_near_miss=9) == "bad"


def test_sphere_table_hit() -> None:
    frames = [
        [
            {"name": "base", "pos": [0.0, 0.0, 0.3]},
            {"name": "wrist", "pos": [0.1, 0.0, 0.002]},  # below clearance
        ]
    ]
    out = analyze_sphere_collisions(
        frames,
        SafetyConfig(table_z=0.0, table_clearance_m=0.008, skip_adjacent=1),
    )
    assert out["n_table"] == 1
    assert out["table_hits"][0]["link"] == "wrist"
    assert out["n_self"] == 0


def test_sphere_self_and_near_miss() -> None:
    # links 0 and 2 (skip_adjacent=1 skips only immediate neighbor index)
    frames = [
        [
            {"name": "a", "pos": [0.0, 0.0, 0.2]},
            {"name": "b", "pos": [0.05, 0.0, 0.2]},
            {"name": "c", "pos": [0.01, 0.0, 0.2]},  # dist(a,c)=0.01 < 0.02 hit
        ]
    ]
    hit = analyze_sphere_collisions(
        frames,
        SafetyConfig(self_min_dist_m=0.02, near_miss_margin_m=0.015, skip_adjacent=1),
    )
    assert hit["n_self"] >= 1
    assert hit["method"] == "sphere"

    frames_near = [
        [
            {"name": "a", "pos": [0.0, 0.0, 0.2]},
            {"name": "b", "pos": [1.0, 0.0, 0.2]},
            {"name": "c", "pos": [0.03, 0.0, 0.2]},  # 30mm: near if hit=20, margin=15
        ]
    ]
    near = analyze_sphere_collisions(
        frames_near,
        SafetyConfig(self_min_dist_m=0.02, near_miss_margin_m=0.015, skip_adjacent=1),
    )
    assert near["n_self"] == 0
    assert near["n_near_miss"] >= 1
    assert near["near_miss"][0]["clearance_m"] > 0


def test_hull_self_hit() -> None:
    frames = [
        [
            {
                "name": "l0",
                "radius": 0.02,
                "points": [(0.0, 0.0, 0.2), (0.01, 0.0, 0.2)],
            },
            {
                "name": "l1",
                "radius": 0.02,
                "points": [(0.5, 0.0, 0.2)],
            },
            {
                "name": "l2",
                "radius": 0.02,
                "points": [(0.005, 0.0, 0.2)],
            },
        ]
    ]
    out = analyze_hull_collisions(
        frames,
        SafetyConfig(method="hull", self_min_dist_m=0.02, skip_adjacent=1),
    )
    assert out["method"] == "hull"
    assert out["n_self"] >= 1


def test_analyze_traj_safety_wrap() -> None:
    frames = [{"links": [{"name": "a", "pos": [0.0, 0.0, 0.5]}]}]
    out = analyze_traj_safety(frames, config={"method": "sphere"})
    assert out["schema"] == "sensors_traj_safety_v1"
    assert out["ok"] is True
    assert out["grade"] == "good"
    assert out["config"]["method"] == "sphere"
