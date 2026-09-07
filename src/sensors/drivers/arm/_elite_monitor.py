"""Elite EC monitor open/close helpers with reconnect retries.

The bundled ``elirobots`` SDK ``__first_connect`` does a one-shot ``recv(4)`` on
TCP 8056 and ``struct.unpack``s without checking length. After disconnect or
long idle, that handshake often gets an empty/short read and the monitor
thread dies with ``struct.error``. We cannot fix the controller; we retry
creating ``EC`` + monitor until ``machinePos`` is ready.
"""

from __future__ import annotations

import time
from typing import Any, Callable


def machine_pos_ready(robot: Any, *, num_joints: int) -> bool:
    pos = getattr(getattr(robot, "monitor_info", None), "machinePos", None)
    return bool(pos is not None and len(pos) >= num_joints and pos[0] is not None)


def monitor_thread_alive(robot: Any) -> bool:
    th = getattr(robot, "monitor_thread", None)
    if th is None:
        return False
    try:
        return bool(th.is_alive())
    except Exception:  # noqa: BLE001
        return False


def stop_monitor(robot: Any, *, join_timeout_s: float = 2.0) -> None:
    """Best-effort stop; never raises to callers."""
    if robot is None:
        return
    try:
        robot.monitor_run_state = False
    except Exception:  # noqa: BLE001
        pass
    try:
        sock = getattr(robot, "sock_monitor", None)
        if sock is not None:
            try:
                sock.close()
            except Exception:  # noqa: BLE001
                pass
    except Exception:  # noqa: BLE001
        pass
    try:
        th = getattr(robot, "monitor_thread", None)
        if th is not None and getattr(th, "is_alive", lambda: False)():
            th.join(timeout=max(0.05, float(join_timeout_s)))
    except Exception:  # noqa: BLE001
        pass
    try:
        if hasattr(robot, "monitor_thread_stop"):
            # Prefer timed join above; call official stop only if already dead.
            th = getattr(robot, "monitor_thread", None)
            if th is None or not th.is_alive():
                robot.monitor_run_state = False
    except Exception:  # noqa: BLE001
        pass


def open_ec_with_monitor(
    *,
    robot_ip: str,
    num_joints: int,
    monitor_wait_s: float = 10.0,
    monitor_retries: int = 3,
    monitor_retry_backoff_s: float = 1.0,
    post_close_cooldown_s: float = 0.3,
    sensor_id: str = "arm",
    log: Callable[[str], None] | None = print,
) -> Any:
    """Create ``EC``, start monitor, wait for ``machinePos``; retry on handshake death.

    Raises:
        ImportError: elite SDK missing.
        RuntimeError / TimeoutError: retries exhausted.
    """
    from elite import EC  # type: ignore

    retries = max(1, int(monitor_retries))
    wait_s = max(0.5, float(monitor_wait_s))
    backoff = max(0.0, float(monitor_retry_backoff_s))
    cooldown = max(0.0, float(post_close_cooldown_s))
    last_err: str | None = None
    _log = log or (lambda _m: None)

    for attempt in range(1, retries + 1):
        robot: Any = None
        try:
            _log(f"[elite-monitor] {sensor_id} connect attempt {attempt}/{retries} ip={robot_ip}")
            robot = EC(ip=robot_ip, auto_connect=True)
            if not hasattr(robot, "monitor_thread_run"):
                raise RuntimeError("elite.EC missing monitor_thread_run")
            robot.monitor_thread_run()

            deadline = time.perf_counter() + wait_s
            while time.perf_counter() < deadline:
                if machine_pos_ready(robot, num_joints=num_joints):
                    _log(f"[elite-monitor] {sensor_id} machinePos ready (attempt {attempt})")
                    return robot
                if not monitor_thread_alive(robot):
                    raise RuntimeError(
                        "8056 monitor thread died during handshake "
                        "(often short/empty MessageSize recv after reconnect/idle)"
                    )
                time.sleep(0.05)

            raise TimeoutError(
                f"monitor_info.machinePos not ready within {wait_s}s (ip={robot_ip})"
            )
        except Exception as e:  # noqa: BLE001
            last_err = str(e)
            _log(f"[elite-monitor] {sensor_id} attempt {attempt} failed: {last_err}")
            stop_monitor(robot)
            if attempt >= retries:
                break
            if backoff > 0:
                time.sleep(backoff)

    raise TimeoutError(
        f"{sensor_id}: Elite 8056 monitor handshake failed after {retries} attempt(s) "
        f"(ip={robot_ip}): {last_err or 'unknown'}. "
        f"Wait a few seconds and retry; avoid dual clients on the same IP; "
        f"see docs/elite-monitor-reconnect.md"
    )


def close_ec_monitor(
    robot: Any,
    *,
    join_timeout_s: float = 2.0,
    post_close_cooldown_s: float = 0.3,
) -> None:
    stop_monitor(robot, join_timeout_s=join_timeout_s)
    cool = max(0.0, float(post_close_cooldown_s))
    if cool > 0:
        time.sleep(cool)
