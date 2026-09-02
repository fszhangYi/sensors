# sensors.kinematics

Elite EC FK/IK ported from `autodl-tmp/demo_test` for sensors-dcs hik_dataset cartesian fields.

- `fk_flange_from_joints_rad` / `make_hik_fk_fn` — used by `export-hik-dataset` (**NumPy only**; no SciPy)
- `ik_flange` requires `scipy` (`pip install hik-sensors[kinematics]`) and is imported lazily so FK/export still works without it
