# Trajectory safety SDK（厚 M2 核心）

日期：2026-11-04  
状态：设计落地中（v0：可移植几何分析；mesh FK 采样后续）  
包名：`sensors.traj_safety`  
消费者：`sensors-dcs`（厚 M2 sidecar）→ 随后 `embody_model_eval`（替换 JS 本地实现）  
相关：[`ecosystem.md`](ecosystem.md) · [`../src/sensors/traj_safety/`](../src/sensors/traj_safety/) · DCS [`capability-metrics-plan.md`](../../sensors-dcs/docs/capability-metrics-plan.md)

---

## 1. 目标与非目标

### 目标

- 在 **`sensors` 核心库**提供与 UI / 录制路径无关的轨迹几何安全分析。
- **同一份 Python API** 供：
  - `sensors-dcs` 厚 M2：离线写 `export/quality/traj_quality.json` + `near_miss.json`
  - `embody_model_eval`：逐步放弃 `collision_geom.js` / `advanced_cd.js` 中的可移植部分
- 输入输出 **schema 稳定**，便于 CLI、KPI 汇总与浏览器展示。

### 非目标（v0）

- 不在核心库依赖 Three.js / urdf-loader。
- 不做采集热路径在线碰撞。
- 不把 DCS `episode/` 路径或 embody `compare_result.json` 绑进核心 API（适配器在各 app）。
- 薄 M2（关节/TCP 动力学超速）仍在 `sensors-dcs.export.traj_quality`；本包专注 **几何 + 间隙**，可与薄结果合并 grade。

---

## 2. 从 embody 迁出什么

| Embody 模块 | 可移植？ | 迁入策略 |
|-------------|---------|----------|
| `analyzeHullCollisions` | ✅ 纯数值 | → `traj_safety.hull.analyze_hull_collisions` |
| `analyzeCollisionsLegacy`（sphere） | ✅ | → `traj_safety.sphere.analyze_sphere_collisions` |
| `analyzeSmoothness` / limits | ✅（可选） | 后期；与薄 M2 动力学部分重叠，避免双源 |
| `tcp_metrics` / `batch_score` | ✅ 但属评测门控 | 不进本包；embody 评测保留或另包 |
| `sampleCollisionClouds` | ❌ Three.js | 仍属 embody / 未来 `sensors` mesh 采样器（trimesh） |
| 点云可视化 | ❌ | 仅 embody |

**现状要点（迁出时必须对齐或显式修正）：**

- 桌面 = 无限平面 `z = table_z` + `table_clearance_m`，**不是**桌子 mesh。
- Self-collision = 点云/球心最小距离 vs `self_min_dist_m`（+ hull 半径垫）。
- 无正式 near-miss 等级；厚 M2 在 SDK 中新增：间隙 ∈ `(hit, near]` → `near_miss`。
- Embody 当前 `parseCollision=false`，采样的是 **visual** mesh；Python mesh 路径应默认优先 collision 几何。
- Root 旋转与桌面轴（Y-up vs Z 平面）在 embody 有已知不一致；SDK 用显式 `up_axis` / `table_normal` 配置，默认 `z` 以兼容现有 JS 数值，文档标明风险。

---

## 3. 分层

```text
┌─────────────────────────────────────────────────────────┐
│  sensors-dcs adapters     embody adapters / UI          │
│  (episode jsonl → samples) (compare JSON / Three FK)    │
└─────────────────────┬───────────────────┬───────────────┘
                      │                   │
                      ▼                   ▼
              ┌───────────────────────────────────┐
              │     sensors.traj_safety (core)    │
              │  types · sphere · hull · grade    │
              │  (optional) link_poses via kin    │
              └───────────────────┬───────────────┘
                                  │
              ┌───────────────────▼───────────────┐
              │ sensors.cs66_kinematics (FK only) │
              │  — Phase B+: link chain / capsules│
              └───────────────────────────────────┘
```

**规则（ecosystem）：** 共享几何/门控算法只进 `sensors`；URDF 浏览与 jog UI 留在 `sensors-arm`；DCS / embody 只做 I/O 适配。

---

## 4. 公共 API（v0）

### 4.1 输入（中立）

```python
# 一帧：若干 link 的球心（sphere）或点云（hull）
LinkPose = {"name": str, "pos": (x, y, z)}           # meters
LinkCloud = {"name": str, "points": [(x,y,z), ...], "radius": float}

TrajectorySafetyInput = {
  "schema": "sensors_traj_safety_input_v1",
  "frames": [  # index i
     {"links": [LinkPose, ...]}           # sphere
     # or {"clouds": [LinkCloud, ...]}    # hull
  ],
  "config": SafetyConfig,
}
```

`SafetyConfig`（与 embody `robots.json` collision 块对齐）：

| 字段 | 默认 | 含义 |
|------|------|------|
| `method` | `sphere` \| `hull` | 分析器 |
| `table_z` | `0.0` | 桌面平面高度 (m) |
| `table_clearance_m` | `0.008` | 低于此带算 table hit |
| `self_min_dist_m` | `0.02` | 自碰阈值 |
| `skip_adjacent` | `1` | 跳过相邻 link 下标 |
| `near_miss_margin_m` | `0.015` | 自碰：hit 阈值之上的 near 带 |
| `max_events` | `40` | 事件列表封顶 |
| `up_axis` | `z` | 桌面法向轴（`x`/`y`/`z`） |

### 4.2 输出

```python
{
  "schema": "sensors_traj_safety_v1",
  "method": "sphere"|"hull",
  "grade": "good"|"warn"|"bad",   # bad=有 collision hit；warn=仅 near_miss；good=无
  "n_table": int,
  "n_self": int,
  "n_near_miss": int,
  "table_hits": [{i, link, z|axis_value, method}],
  "self_hits": [{i, a, b, dist_mm, method}],
  "near_miss": [{i, a, b, dist_mm, clearance_m, method}],
  "config": {...},
}
```

Grade 合成（厚 M2 KPI）：

- `bad`：`n_table > 0` 或 `n_self > 0`
- `warn`：仅有 `near_miss`
- `good`：全无

与薄 M2 合并时（DCS）：动力学 warn ∪ 几何 warn → warn；任一 bad → bad。

### 4.3 Python 入口

```python
from sensors.traj_safety import (
    SafetyConfig,
    analyze_sphere_collisions,
    analyze_hull_collisions,
    analyze_traj_safety,
    grade_from_counts,
)
```

`analyze_traj_safety(frames, *, config)` 按 `config.method` 分派。

---

## 5. 分期

| 阶段 | 内容 | 状态 |
|------|------|------|
| **A · v0** | 文档 + sphere/hull 纯数值 + schema + 测试 | **done** |
| **B** | MDH/DH **连杆位姿链** + YAML safety profile；DCS `traj-safety` 写 thick sidecar + UI/KPI | **done** |
| **C** | STL 点云采样（无 trimesh）+ `elite_cs66_mesh` profile；embody 消费另开 | **SDK done** / embody pending |
| **D** | embody 删除重复 JS 数值逻辑，只留可视化 | later |

---

## 6. 与薄 M2 / DCS 的边界

| | 薄 M2 | 厚 M2（本 SDK） |
|--|------|----------------|
| 代码位置 | `sensors-dcs.export.traj_quality` | `sensors.traj_safety` + DCS 适配 |
| 输入 | `states/arm*.jsonl` | link 位姿/点云序列（由 FK/mesh 适配器生成） |
| 输出 | `export/quality/traj_quality.json` (`kind=thin`) | 同文件可升 `kind=thick` 或并列 `near_miss.json` |
| 依赖 | 无 | 核心库；B+ 依赖 `cs66_kinematics` |

一键后处理：厚分析仍为 **可选 / independent**，不拖慢默认三步。

---

## 7. 测试约定

- `tests/test_traj_safety_*.py`，NumPy-only，不强制 SciPy。
- 固定几何夹具：已知桌面穿透、已知自碰、near-miss 带、干净轨迹。
- 与 embody 数值对齐：同一 `linkSeries` / `hullSeries` JSON 夹具，sphere/hull 计数一致（允许 float 容差）。

---

## 8. 开放问题

1. 桌面法向默认 `z` 还是统一改 `y`（与 Three Y-up 对齐）？→ v0 默认 `z` + 可配置。
2. Capsule 几何放 `cs66_kinematics/profiles/*.yaml` 还是独立 `traj_safety/profiles/`？→ 建议 **独立 safety profile** 引用 `kinematics_profile_id`，避免污染 FK YAML。
3. Pred vs GT：embody 安全分析用 pred；DCS 厚 M2 用 **实测 arm_read**。适配器各自选择序列，核心不管来源标签（可在 meta 里带 `source`）。
