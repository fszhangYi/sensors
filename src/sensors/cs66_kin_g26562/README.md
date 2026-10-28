# sensors.cs66_kinematics

Sibling of `sensors.kinematics`（原包不动）。型号数据只在 `profiles/*.yaml`。

```text
cs66_kinematics/
  profiles/
    elite_ec616.yaml
    elite_cs66.yaml
  yaml_profile.py
  config.py / constants.py / model.py
  dh.py / fk.py / ik.py / teach.py
  __init__.py
```

（已去掉 `robot.xml` / `robot_xml.py` / `rmp.py` — 非 YAML FK/IK 主路径所需。）

选用：`get_kinematic_config("elite_ec616")` 或环境变量 `CS66_KINEMATICS_PROFILE`。

详见 `YAML_MIGRATE.md`。
