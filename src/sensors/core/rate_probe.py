"""Shared helpers for driver-native rate probes (not sample read()/write() loops)."""

from __future__ import annotations

import math
import time
from typing import Any

MIN_SAMPLES = 3
MIN_DURATION_S = 0.5


def clamp_duration(duration_s: float) -> float:
    return max(MIN_DURATION_S, float(duration_s))


def measure_rate_from_times(times: list[float]) -> float | None:
    if len(times) < MIN_SAMPLES:
        return None
    span = times[-1] - times[0]
    if span <= 0:
        return None
    return (len(times) - 1) / span


def cap_from_measured(measured: float, *, margin: float = 0.95) -> float:
    return max(0.2, math.floor(float(measured) * margin * 10) / 10)


def timing_stats_ms(times: list[float]) -> dict[str, float | None]:
    if len(times) < 2:
        return {"dt_ms_p50": None, "dt_ms_p95": None, "dt_ms_mean": None}
    dts = sorted((times[i] - times[i - 1]) * 1000.0 for i in range(1, len(times)))
    mean = sum(dts) / len(dts)
    mid = dts[len(dts) // 2]
    p95 = dts[min(len(dts) - 1, int(len(dts) * 0.95))]
    return {
        "dt_ms_p50": round(mid, 3),
        "dt_ms_p95": round(p95, 3),
        "dt_ms_mean": round(mean, 3),
    }


def rate_probe_unsupported(kind: str, *, reason: str | None = None) -> dict[str, Any]:
    return {
        "ok": False,
        "unsupported": True,
        "error": reason or f"rate probe not supported for kind={kind}",
        "method": "unsupported",
    }


def rate_probe_fail(
    *,
    error: str,
    method: str,
    samples: int = 0,
    errors: int = 0,
    duration_s: float = 0.0,
    dry_run: bool | None = None,
    last_error: str | None = None,
    **extra: Any,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "ok": False,
        "error": error,
        "samples": samples,
        "errors": errors,
        "duration_s": round(duration_s, 3),
        "method": method,
    }
    if dry_run is not None:
        out["dry_run"] = dry_run
    if last_error:
        out["last_error"] = last_error
    out.update(extra)
    return out


def rate_probe_ok(
    times: list[float],
    *,
    method: str,
    errors: int = 0,
    t0: float | None = None,
    dry_run: bool | None = None,
    last_error: str | None = None,
    **extra: Any,
) -> dict[str, Any]:
    elapsed = max(1e-9, time.perf_counter() - (t0 if t0 is not None else times[0]))
    measured = measure_rate_from_times(times)
    if measured is None:
        span = times[-1] - times[0] if len(times) >= 2 else 0.0
        measured = len(times) / elapsed if span <= 0 and len(times) >= MIN_SAMPLES else None
    if measured is None:
        return rate_probe_fail(
            error=last_error or "rate probe produced too few samples",
            method=method,
            samples=len(times),
            errors=errors,
            duration_s=elapsed,
            dry_run=dry_run,
            last_error=last_error,
            **extra,
        )
    stats = timing_stats_ms(times)
    out: dict[str, Any] = {
        "ok": True,
        "samples": len(times),
        "errors": errors,
        "duration_s": round(elapsed, 3),
        "measured_hz": round(float(measured), 3),
        "read_cap_hz": cap_from_measured(measured),
        "method": method,
        **stats,
    }
    if dry_run is not None:
        out["dry_run"] = dry_run
    if last_error:
        out["last_error"] = last_error
    out.update(extra)
    return out


def default_read_probe_duration_s(kind: str) -> float:
    k = str(kind or "").lower()
    if k == "realsense":
        return 3.0
    if k in ("ft", "gello"):
        return 5.0
    return 10.0
