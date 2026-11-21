#!/usr/bin/env python3
"""Pack CS66 kinematics outbound zip: runnable SDK + build docs (comments stripped)."""

from __future__ import annotations

import ast
import shutil
import time
import zipfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src" / "sensors" / "cs66_kin_g26562"
OUT_ROOT = ROOT / "release"

SDK_PY = (
    "__init__.py",
    "config.py",
    "constants.py",
    "dh.py",
    "fk.py",
    "ik.py",
    "model.py",
    "teach.py",
    "yaml_profile.py",
)


def _stamp() -> str:
    return time.strftime("%Y%m%d-%H%M%S", time.localtime())


def _copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _strip_python(src: str) -> str:
    """Remove comments + docstrings via AST unparse (compact runnable code)."""
    tree = ast.parse(src)

    def _is_str_expr(stmt: ast.stmt) -> bool:
        return (
            isinstance(stmt, ast.Expr)
            and isinstance(stmt.value, ast.Constant)
            and isinstance(stmt.value.value, str)
        )

    def _clear_module(body: list[ast.stmt]) -> None:
        i = 0
        while (
            i < len(body)
            and isinstance(body[i], ast.ImportFrom)
            and body[i].module == "__future__"
        ):
            i += 1
        while i < len(body) and _is_str_expr(body[i]):
            body.pop(i)

    def _clear_block(body: list[ast.stmt]) -> None:
        while body and _is_str_expr(body[0]):
            body.pop(0)

    _clear_module(tree.body)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            _clear_block(node.body)
    text = ast.unparse(tree)
    lines: list[str] = []
    blank = 0
    for line in text.splitlines():
        if line.strip() == "":
            blank += 1
            if blank <= 1:
                lines.append("")
        else:
            blank = 0
            lines.append(line.rstrip())
    return "\n".join(lines).strip() + "\n"


def _patch_outbound_python(name: str, text: str) -> str:
    if name == "yaml_profile.py":
        text = text.replace(
            'DEFAULT_PROFILE_ID = "elite_ec616"',
            'DEFAULT_PROFILE_ID = "elite_cs66"',
        )
        text = text.replace(
            "Profile files live under ``cs66_kinematics/profiles/*.yaml``.",
            "Profile files live under profiles/*.yaml.",
        )
    return text


def _write_slim_profile(src_yaml: Path, dst_yaml: Path) -> None:
    data = yaml.safe_load(src_yaml.read_text(encoding="utf-8"))
    # Drop verbose narrative; keep machine-usable fields only
    data["notes"] = "G26562 CS66 unit MDH pack"
    if isinstance(data.get("install"), dict):
        data["install"]["note"] = "floor; tilt=0; rot_z=0; R_install=I"
    # Stable key order for readability
    preferred = [
        "id",
        "brand",
        "product",
        "supported",
        "notes",
        "manipulator_type",
        "dh_convention",
        "link_length_mm",
        "mdh",
        "joint_offset_deg",
        "joint_direction",
        "default_joint_angle_mode",
        "base_cs_x",
        "base_cs_z",
        "install",
        "default_tcp_xyz_m",
        "soft_limits_deg",
        "joint_vel_max_deg_s",
        "joint_vel_min_deg_s",
        "joint_accel_max_deg_s2",
        "joint_effort_max_nm",
        "cartesian",
        "teach_poses",
    ]
    ordered = {k: data[k] for k in preferred if k in data}
    for k, v in data.items():
        if k not in ordered:
            ordered[k] = v
    dst_yaml.parent.mkdir(parents=True, exist_ok=True)
    dst_yaml.write_text(
        yaml.safe_dump(
            ordered,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
            width=100,
        ),
        encoding="utf-8",
    )


def _write_sdk(stage: Path) -> Path:
    pkg = stage / "sdk" / "cs66_kin"
    pkg.mkdir(parents=True, exist_ok=True)
    for name in SDK_PY:
        src = SRC / name
        if not src.is_file():
            raise SystemExit(f"missing SDK file: {src}")
        raw = src.read_text(encoding="utf-8")
        slim = _strip_python(_patch_outbound_python(name, raw))
        (pkg / name).write_text(slim, encoding="utf-8", newline="\n")
    _write_slim_profile(
        SRC / "profiles" / "elite_cs66.yaml",
        pkg / "profiles" / "elite_cs66.yaml",
    )
    return pkg


def _write_smoke(stage: Path) -> None:
    ex = stage / "examples"
    ex.mkdir(parents=True, exist_ok=True)
    (ex / "smoke_fk_ik.py").write_text(
        '''#!/usr/bin/env python3
"""Minimal FK/IK smoke for outbound cs66_kin SDK."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sdk"))

import numpy as np
from cs66_kin import fk_flange, get_profile, ik_flange


def main() -> int:
    prof = get_profile("elite_cs66")
    assert prof.supported
    q = list(prof.teach_poses[0].joint_deg)
    T = fk_flange(q, joint_angle_mode="machine_deg")
    xyz = T[:3, 3] * 1000.0
    target = np.asarray(prof.teach_poses[0].position_mm, dtype=float)
    err = float(np.linalg.norm(xyz - target))
    print(f"FK pose1 |xyz err| = {err:.4f} mm")
    q_ik = ik_flange(T, np.asarray(q, dtype=float), return_details=True)
    ok = bool(getattr(q_ik, "success", True))
    print(f"IK success={ok} nfev={getattr(q_ik, 'nfev', '?')}")
    if err > 0.05:
        raise SystemExit(f"FK residual too large: {err} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
''',
        encoding="utf-8",
        newline="\n",
    )


def main() -> int:
    if not SRC.is_dir():
        raise SystemExit(f"missing {SRC}")

    stamp = _stamp()
    name = f"cs66-kinematics-build-outbound-{stamp}"
    stage = OUT_ROOT / name
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True)

    _write_sdk(stage)
    _write_smoke(stage)

    (stage / "requirements.txt").write_text(
        "numpy>=1.23\nscipy>=1.10\nPyYAML>=6\n",
        encoding="utf-8",
        newline="\n",
    )

    for fn in (
        "构建指南.md",
        "构建指南.docx",
        "现场采集空白表.md",
        "现场采集空白表.docx",
        "交叉验证.md",
    ):
        src = SRC / fn
        if src.is_file():
            _copy(src, stage / fn)

    mdh = SRC / "示教器信息" / "MDH.jpg"
    if mdh.is_file():
        _copy(mdh, stage / "evidence" / "MDH.jpg")

    (stage / "README.md").write_text(
        f"""# CS66 正逆运动学构建外发包

生成时间：{stamp}  
样机：Elite CS66 / G26562  
来源：`sensors/src/sensors/cs66_kin_g26562`（外发时注释已精简）

## 内容

| 路径 | 说明 |
|------|------|
| `sdk/cs66_kin/` | **可运行** FK/IK SDK（NumPy + SciPy） |
| `sdk/cs66_kin/profiles/elite_cs66.yaml` | 已验收 MDH / 限位 / teach_poses |
| `examples/smoke_fk_ik.py` | 冒烟：pose#1 FK 残差 + IK |
| `requirements.txt` | `numpy` / `scipy` / `PyYAML` |
| `构建指南.docx` | 构建主读本（嵌图） |
| `构建指南.md` / `现场采集空白表.*` / `交叉验证.md` | 文档与表格 |
| `evidence/MDH.jpg` | 示教器 MDH 原图 |

## 运行 SDK

```bash
python -m venv .venv
# Windows: .venv\\Scripts\\activate
source .venv/bin/activate
pip install -r requirements.txt
python examples/smoke_fk_ik.py
```

代码示例：

```python
import sys
sys.path.insert(0, "sdk")
from cs66_kin import fk_flange, ik_flange, get_profile

import numpy as np
q = [174.316, -105.899, 106.028, -158.769, 267.136, 0.344]
T = fk_flange(q, joint_angle_mode="machine_deg")
q2 = ik_flange(T, np.asarray(q, dtype=float))
```

默认 profile：`elite_cs66`（可用环境变量 `CS66_KINEMATICS_PROFILE` 覆盖）。

## 构建流程（现场）

1. 读 `构建指南.docx`  
2. 填 `现场采集空白表`  
3. 对照 `profiles/elite_cs66.yaml` 写本机 YAML  
4. 用 `smoke_fk_ik.py` / 自有脚本做 FK/IK 验收  

## 未包含

- 完整 `hik-sensors` 驱动栈、硬件 SDK  
- `示教器信息/` 整目录原图、HEIC、`guide_assets`（图已在 docx）  
- EC616 profile、docx 生成脚本  

## 验收摘要（G26562）

16 组 teach：FK 约 0.001–0.005 mm；姿态约 0.001°；IK 最大关节误差范数约 0.007°。
""",
        encoding="utf-8",
        newline="\n",
    )

    zip_path = OUT_ROOT / f"{name}.zip"
    if zip_path.exists():
        zip_path.unlink()
    def _iter_files() -> list[Path]:
        files: list[Path] = []
        for f in sorted(stage.rglob("*")):
            if not f.is_file():
                continue
            if "__pycache__" in f.parts or f.suffix == ".pyc":
                continue
            files.append(f)
        return files

    files = _iter_files()
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for f in files:
            zf.write(f, f.relative_to(stage).as_posix())

    n = len(files)
    total = sum(p.stat().st_size for p in files)
    print(f"staged: {stage}")
    print(f"zip:    {zip_path}")
    print(f"files:  {n}")
    print(f"stage:  {total / (1024 * 1024):.1f} MB")
    print(f"zip:    {zip_path.stat().st_size / (1024 * 1024):.1f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
