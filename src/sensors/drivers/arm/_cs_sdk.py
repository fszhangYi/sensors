"""Helpers for Elite CS series SDK (``elite_cs_sdk``) — not EC ``elirobots``.

CS stack (distinct from EC ``elite.EC`` / ports 54321+8056):
  - Control: ``EliteDriver.writeServoj`` (joint rad)
  - State: Primary **30001** / RTSI **30004** (``RtsiIOInterface.getActualJointPositions``)
  - Package: https://github.com/Elite-Robots/Elite_Robots_CS_SDK_Python
"""

from __future__ import annotations

import time
from pathlib import Path
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


def resolve_external_control_script(script_file_path: str = "") -> str:
    """Resolve Elite ``external_control.script`` path for ``EliteDriverConfig``.

    Official examples always set ``script_file_path`` to the file shipped inside
    the ``elite_cs_sdk`` package. An empty path makes ``EliteDriver`` raise
    ``Elite script file '' doesn't exists`` (surfaced as ``unknow code: …``).
    """
    explicit = str(script_file_path or "").strip()
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_file():
            raise FileNotFoundError(
                f"CS external control script not found: {path} "
                "(params.script_file_path)"
            )
        return str(path.resolve())

    sdk = import_elite_cs_sdk()
    pkg_file = getattr(sdk, "__file__", None)
    if not pkg_file:
        raise FileNotFoundError(
            "elite_cs_sdk has no __file__; cannot locate external_control.script"
        )
    bundled = Path(pkg_file).resolve().parent / "external_control.script"
    if not bundled.is_file():
        raise FileNotFoundError(
            f"bundled CS script missing: {bundled}. "
            "Reinstall elite_cs_sdk wheel or set params.script_file_path."
        )
    return str(bundled)


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
    # EliteDriver::init always reads this file after Primary connect.
    cfg.script_file_path = resolve_external_control_script(script_file_path)
    try:
        cfg.servoj_time = float(servoj_time)
        cfg.servoj_lookahead_time = float(servoj_lookahead_time)
        cfg.servoj_gain = int(servoj_gain)
        cfg.stopj_acc = float(stopj_acc)
    except Exception:  # noqa: BLE001
        pass
    # EliteDriver::init already sends the script when headless_mode=True.
    # Motion readiness (power / brake / reverse TCP) is handled in ensure_cs_motion_ready().
    return sdk.EliteDriver(cfg)


def ensure_cs_motion_ready(
    driver: Any,
    *,
    headless_mode: bool = True,
    power_on: bool = True,
    timeout_s: float = 15.0,
    poll_s: float = 0.05,
) -> dict[str, Any]:
    """Make reverse/trajectory/script channels ready for ``writeServoj``.

    Mirrors Elite examples (``example_servoj_plan.py``):
      primary.powerOn → brakeRelease → (headless) sendExternalControlScript
      → wait until ``isRobotConnected()``.

    ``writeServoj`` returns false when the robot has not yet connected back to
    the PC reverse ports (default 50001 / 50003 / 50004).
    """
    if driver is None:
        return {"ok": False, "error": "CS driver is None", "connected": False}

    steps: list[str] = []
    if power_on and hasattr(driver, "primaryPort"):
        try:
            primary = driver.primaryPort()
        except Exception as e:  # noqa: BLE001
            return {
                "ok": False,
                "error": f"CS primaryPort() failed: {e}",
                "connected": False,
                "steps": steps,
            }
        if hasattr(primary, "powerOn"):
            try:
                ok_pwr = bool(primary.powerOn())
            except Exception as e:  # noqa: BLE001
                return {
                    "ok": False,
                    "error": f"CS powerOn failed: {e}",
                    "connected": False,
                    "steps": steps,
                }
            steps.append(f"powerOn={'ok' if ok_pwr else 'false'}")
            if not ok_pwr:
                return {
                    "ok": False,
                    "error": "CS powerOn returned false (check E-stop / teach pendant)",
                    "connected": False,
                    "steps": steps,
                }
        if hasattr(primary, "brakeRelease"):
            try:
                ok_brk = bool(primary.brakeRelease())
            except Exception as e:  # noqa: BLE001
                return {
                    "ok": False,
                    "error": f"CS brakeRelease failed: {e}",
                    "connected": False,
                    "steps": steps,
                }
            steps.append(f"brakeRelease={'ok' if ok_brk else 'false'}")
            if not ok_brk:
                return {
                    "ok": False,
                    "error": "CS brakeRelease returned false",
                    "connected": False,
                    "steps": steps,
                }
        time.sleep(0.5)

    # EliteDriver ctor may have sent ExternalControl *before* power/brake.
    # Always (re)send after power so the running script matches a ready robot.
    # If already connected, skip resend to avoid tearing down live reverse sockets.
    connected = bool(getattr(driver, "isRobotConnected", lambda: False)())
    if headless_mode and hasattr(driver, "sendExternalControlScript"):
        if connected:
            steps.append("isRobotConnected=preexisting")
        else:
            try:
                sent = bool(driver.sendExternalControlScript())
            except Exception as e:  # noqa: BLE001
                return {
                    "ok": False,
                    "error": f"CS sendExternalControlScript failed: {e}",
                    "connected": False,
                    "steps": steps,
                }
            steps.append(f"sendExternalControlScript={'ok' if sent else 'false'}")
            if not sent:
                return {
                    "ok": False,
                    "error": (
                        "CS sendExternalControlScript returned false "
                        "(Primary script send failed; check robot mode / remote control)"
                    ),
                    "connected": False,
                    "steps": steps,
                }

    deadline = time.perf_counter() + max(0.5, float(timeout_s))
    while time.perf_counter() < deadline:
        connected = bool(getattr(driver, "isRobotConnected", lambda: False)())
        if connected:
            steps.append("isRobotConnected=true")
            return {"ok": True, "connected": True, "steps": steps}
        time.sleep(max(0.01, float(poll_s)))

    hint = (
        "Robot did not connect reverse channels in time. "
        "Need ExternalControl running and PC reachable on reverse/trajectory/"
        "script_command ports (default 50001/50003/50004). "
        "Check Windows firewall, local_ip, and that the arm is Remote/Auto."
    )
    if not headless_mode:
        hint = (
            "headless_mode=false: start the External Control program on the "
            "teach pendant, then Arm again. " + hint
        )
    return {
        "ok": False,
        "error": hint,
        "connected": False,
        "steps": steps,
        "timeout_s": float(timeout_s),
    }


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


def resolve_rtsi_recipe_path(path: str) -> str:
    """Resolve RTSI recipe path (absolute, cwd, or sensors-arm/configs)."""
    raw = str(path or "").strip()
    if not raw:
        raise FileNotFoundError("empty RTSI recipe path")
    candidates: list[Path] = [Path(raw).expanduser()]
    if not candidates[0].is_absolute():
        cwd = Path.cwd()
        candidates.append(cwd / raw)
        candidates.append(cwd / "configs" / "rtsi" / Path(raw).name)
        # sensors-arm package layout: <repo>/configs/rtsi/...
        here = Path(__file__).resolve()
        for parent in here.parents:
            if (parent / "configs" / "sensors_multi.yaml").is_file():
                candidates.append(parent / raw)
                candidates.append(parent / "configs" / "rtsi" / Path(raw).name)
                break
    for cand in candidates:
        try:
            if cand.is_file():
                return str(cand.resolve())
        except OSError:
            continue
    raise FileNotFoundError(
        f"RTSI recipe not found: {raw!r} (tried cwd and sensors-arm/configs/rtsi)"
    )


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
    out_path = resolve_rtsi_recipe_path(out)
    inp_raw = str(input_recipe_file or "").strip() or out
    inp_path = resolve_rtsi_recipe_path(inp_raw)
    rtsi = sdk.RtsiIOInterface(out_path, inp_path, float(frequency))
    if not rtsi.connect(str(robot_ip)):
        raise RuntimeError(
            f"CS RTSI connect failed ip={robot_ip} "
            f"(output={out_path}, input={inp_path}, port 30004 reachable?)"
        )
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
