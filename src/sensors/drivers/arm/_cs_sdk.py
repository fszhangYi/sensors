"""Helpers for Elite CS series SDK (``elite_cs_sdk``) — not EC ``elirobots``.

CS stack (distinct from EC ``elite.EC`` / ports 54321+8056):
  - Control: ``EliteDriver.writeServoj`` (joint rad)
  - State: Primary **30001** / RTSI **30004** (``RtsiIOInterface.getActualJointPositions``)
  - Package: https://github.com/Elite-Robots/Elite_Robots_CS_SDK_Python
"""

from __future__ import annotations

from typing import Any


def import_elite_cs_sdk() -> Any:
    """Import ``elite_cs_sdk`` or raise ImportError with install hint."""
    try:
        import elite_cs_sdk as sdk  # type: ignore
    except ImportError as e:
        raise ImportError(
            "Elite CS SDK required (package elite_cs_sdk). "
            "Build/install from Elite-Robots/Elite_Robots_CS_SDK_Python "
            "(see doc/BuildGuide). Not the same as PyPI elirobots (EC series)."
        ) from e
    return sdk


def open_cs_driver(
    *,
    robot_ip: str,
    headless_mode: bool = True,
    local_ip: str = "",
    script_file_path: str = "",
    servoj_time: float = 0.008,
    servoj_lookahead_time: float = 0.1,
    servoj_gain: int = 300,
    stopj_acc: float = 8.0,
) -> Any:
    """Construct and return ``EliteDriver`` for CS controller."""
    sdk = import_elite_cs_sdk()
    cfg = sdk.EliteDriverConfig()
    cfg.robot_ip = str(robot_ip)
    cfg.headless_mode = bool(headless_mode)
    if local_ip:
        cfg.local_ip = str(local_ip)
    if script_file_path:
        cfg.script_file_path = str(script_file_path)
    try:
        cfg.servoj_time = float(servoj_time)
        cfg.servoj_lookahead_time = float(servoj_lookahead_time)
        cfg.servoj_gain = int(servoj_gain)
        cfg.stopj_acc = float(stopj_acc)
    except Exception:  # noqa: BLE001
        pass
    driver = sdk.EliteDriver(cfg)
    if headless_mode and hasattr(driver, "sendExternalControlScript"):
        try:
            driver.sendExternalControlScript()
        except Exception:  # noqa: BLE001
            pass
    return driver


def close_cs_driver(driver: Any | None, *, wait_ms: int = 2000) -> None:
    if driver is None:
        return
    try:
        if hasattr(driver, "stopControl"):
            driver.stopControl(wait_ms)
    except Exception:  # noqa: BLE001
        pass
    try:
        if hasattr(driver, "writeIdle"):
            driver.writeIdle(100)
    except Exception:  # noqa: BLE001
        pass


def open_cs_rtsi(
    *,
    robot_ip: str,
    output_recipe_file: str,
    input_recipe_file: str = "",
    frequency: float = 125.0,
) -> Any | None:
    """Optional RTSI IO for joint feedback. Returns None if recipes unset."""
    out = str(output_recipe_file or "").strip()
    if not out:
        return None
    sdk = import_elite_cs_sdk()
    inp = str(input_recipe_file or "").strip() or out
    rtsi = sdk.RtsiIOInterface(out, inp, float(frequency))
    if not rtsi.connect(str(robot_ip)):
        raise RuntimeError(f"CS RTSI connect failed ip={robot_ip}")
    return rtsi


def close_cs_rtsi(rtsi: Any | None) -> None:
    if rtsi is None:
        return
    try:
        rtsi.disconnect()
    except Exception:  # noqa: BLE001
        pass


def read_joints_rad_from_rtsi(rtsi: Any, *, n: int = 6) -> list[float]:
    if rtsi is None:
        raise RuntimeError("CS RTSI not open")
    if not hasattr(rtsi, "getActualJointPositions"):
        raise RuntimeError("RtsiIOInterface missing getActualJointPositions")
    pos = list(rtsi.getActualJointPositions())
    if len(pos) < n:
        raise RuntimeError(f"CS RTSI joints short: got {len(pos)} need {n}")
    return [float(x) for x in pos[:n]]
