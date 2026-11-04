# `sensors.traj_safety`

Portable trajectory geometric safety (thick M2 core).

- Design: [`docs/traj-safety-sdk.md`](../../../docs/traj-safety-sdk.md)
- v0: `analyze_sphere_collisions` / `analyze_hull_collisions` / `analyze_traj_safety`
- No Three.js; mesh sampling is Phase C

```python
from sensors.traj_safety import SafetyConfig, analyze_traj_safety

frames = [
    {"links": [
        {"name": "link1", "pos": [0.0, 0.0, 0.2]},
        {"name": "link3", "pos": [0.0, 0.0, 0.05]},
    ]}
]
out = analyze_traj_safety(frames, config=SafetyConfig(method="sphere", table_clearance_m=0.08))
assert out["grade"] in {"good", "warn", "bad"}
```
