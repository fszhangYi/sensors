# hik-sensors

从 `hww/hik_gello`（入口 `MegaCollect.py`）重构的统一传感器栈：配置驱动、协议统一、按 kind 插件扩展。

## 架构

```
configs/default.yaml          # 机台设备清单
src/sensors/
  core/                       # Sensor 协议 / HealthReport / Registry / Config
  drivers/
    bus/                      # 端口映射 socat + by-id
    arm/                      # Follower ZMQ :6001
    gello/                    # Leader Dynamixel
    gripper/                  # DH AG95 Modbus
    camera/                   # RealSense 三角色
    pipeline/                 # 采集 SHM + flag + 远程同步
    extensions/               # F/T、触觉占位
  runtime/manager.py          # SensorManager
  cli/main.py                 # sensors list|probe|read|kinds
```

| MegaCollect | kind | 驱动 | open/read |
|-------------|------|------|-----------|
| 端口映射 | `bus` | `SerialBusSensor` | socat PTY + by-id 状态 |
| 服务端 | `arm` | `FollowerArmSensor` | ZMQ REQ pickle `get_joint_state` |
| 客户端 Gello | `gello` | `GelloLeaderSensor` | Dynamixel GroupSyncRead |
| DH 夹爪 | `gripper` | `DhAg95Sensor` | Modbus RTU 位置反馈 |
| RealSense L/R/M | `realsense` | `RealSenseSensor` | pipeline color(+depth) |
| 数据采集 | `pipeline` | `CollectPipelineSensor` | probe（SHM） |
| （扩展） | `ft` | `ForceTorqueSensor` | HIK 串口 460800，force[3]+torque[3]；dry-run；≠触觉/夹爪力 |
| （扩展） | `tactile` | `PaxiniTactileSensor` | 帕西尼 rest_force + 60 点分力（get_paxini_data） |

每个驱动实现同一协议：`probe()`（只读）/ `open()` / `close()` / `read()`，用 `@register_sensor(kind)` 注册。新增设备：写驱动 + YAML 一条，无需改 Manager。

## 使用

```bash
cd /root/autodl-tmp/sensors
pip install -e ".[dev]"

sensors kinds
sensors list -c configs/default.yaml
sensors probe -c configs/default.yaml -v
sensors probe -c configs/default.yaml --id arm-follower --json
sensors read  -c configs/default.yaml --id arm-follower --dry-run --json
```

Python:

```python
from sensors import SensorManager

mgr = SensorManager.from_yaml("configs/default.yaml")
for report in mgr.probe_all():
    print(report.status, report.sensor_id, report.message)

# 真实采样（需硬件 + 可选依赖）
sample = mgr.read("arm-follower")  # open → read → close
```

可选依赖：`pip install -e ".[serial,zmq,realsense,dynamixel]"`。

## 与 embody_model_eval

本仓库是独立 SDK/运行时，**尚未**接到 Web 状态页。后续应调用 `SensorManager.probe_all()` / `read()`（或包一层 HTTP），而不是在前端堆静态占位。

## License

Apache-2.0
