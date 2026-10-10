# Elite CS vs EC drivers

| Kind | Series | SDK | Control | State ports |
|---|---|---|---|---|
| `arm_read` / `arm_write` | **EC** (EC616, EC66, …) | PyPI `elirobots` → `elite.EC` | `TT_init` + `TT_add_joint` (**deg**) | TCP **54321**, monitor **8056** |
| `arm_cs_read` / `arm_cs_write` | **CS** (CS66 / ES66, …) | `elite_cs_sdk` (CS SDK Python) | `EliteDriver.writeServoj` (**rad**) | Primary **30001**, RTSI **30004** |

Do **not** point a CS66 IP at `arm_write`. Do **not** use `arm_cs_write` on an EC controller.

## Live CS setup

1. Build/install [Elite_Robots_CS_SDK_Python](https://github.com/Elite-Robots/Elite_Robots_CS_SDK_Python).
2. Controller CS software ≥ 2.13.4 / 2.14.2 (see SDK README).
3. YAML: `kind: arm_cs_write`, `robot_ip`, optional `rtsi_output_recipe` for feedback.
4. `dry_run: false` only on a real CS cell.
