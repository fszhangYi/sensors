# 只改 YAML 能否迁到别的机械臂？

结论分两层。

## 同拓扑六轴（与 EC616/CS66 同类）

**可以只改 YAML。** 复制 `profiles/elite_ec616.yaml` → `profiles/<new_id>.yaml`，填写：

1. `id` / `brand` / `product`
2. 几何：`link_length_mm` **或** `mdh` **或** `dh_table`（设好 `dh_convention`）
3. `joint_offset_deg` / `joint_direction`
4. `soft_limits_deg`、速度加速度、`default_tcp_xyz_m`
5. `teach_poses`（≥8 组关节↔法兰校验点）
6. `supported: true`

然后：

```python
from sensors.cs66_kinematics import get_kinematic_config, make_hik_fk_fn, yaml_migration_checklist, get_profile

assert yaml_migration_checklist(get_profile("my_arm"))["yaml_only_same_topology"]
cfg = get_kinematic_config("my_arm")
fk = make_hik_fk_fn("my_arm")
```

当前检查结果（本仓库）：

| profile | yaml_only_same_topology | 阻塞项 |
|---------|-------------------------|--------|
| `elite_ec616` | **True** | 无 |
| `elite_cs66` | False | `supported: false`；连杆全 0；无 teach_poses |

## 代码里仍锁死的假设（改 YAML 不够）

| 假设 | 位置 |
|------|------|
| 六轴、`manipulator_type == 1` | `fk.py` / `ik.py` / `config.py` |
| `link_lengths_standard` 的 α 角图案固定 | `dh.dh_table_from_link_lengths_m` |
| 关节角语义只有 4 种 mode | `fk.joint_q_rad` |
| 基座轴向枚举 `_AXIS_VEC` | `constants.py`（共享，非每臂） |
| 数值 IK（SciPy least_squares） | `ik.py` |

不同轴数、并联、或非偏置腕且无法用 `dh_table`/`mdh` 表达时，必须改代码。

## 与原 `sensors.kinematics` 的关系

- **零修改**原包；本包是同级副本。
- `elite_ec616.yaml` 与 legacy 在零位与首个示教点上 FK 位置误差为 **0**（已跑通对照）。
