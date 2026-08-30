from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from sensors.core.registry import list_registered_kinds
from sensors.drivers import load_all_drivers
from sensors.runtime.manager import SensorManager


def _default_config() -> Path:
    here = Path(__file__).resolve()
    # src/sensors/cli/main.py → repo root configs/default.yaml
    return here.parents[3] / "configs" / "default.yaml"


def _jsonable(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        return {
            "_ndarray": True,
            "dtype": str(obj.dtype),
            "shape": list(obj.shape),
        }
    if isinstance(obj, dict):
        return {k: _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    return obj


def cmd_list(args: argparse.Namespace) -> int:
    load_all_drivers()
    mgr = SensorManager.from_yaml(args.config, dry_run=args.dry_run)
    print(f"site={mgr.bundle.site}  devices={len(mgr)}  registered_kinds={list_registered_kinds()}")
    for s in mgr:
        ep = s.config.get("endpoint") or s.config.get("port") or s.config.get("serial") or "—"
        print(f"  - {s.id:16} kind={s.kind.value:10} name={s.name}  ep={ep}")
    return 0


def cmd_probe(args: argparse.Namespace) -> int:
    mgr = SensorManager.from_yaml(args.config, dry_run=args.dry_run)
    only = [args.id] if args.id else None
    reports = mgr.probe_all(only=only)
    if args.json:
        print(json.dumps([r.to_dict() for r in reports], ensure_ascii=False, indent=2))
        return 0
    worst = 0
    for r in reports:
        mark = {
            "ok": "OK",
            "warn": "WARN",
            "error": "ERR",
            "offline": "OFF",
            "unknown": "???",
            "unsupported": "N/A",
        }.get(r.status.value, r.status.value)
        print(f"[{mark:3}] {r.sensor_id:16} ({r.kind}) — {r.message}")
        for c in r.checks:
            flag = "✓" if c.ok else "✗"
            crit = "!" if c.critical and not c.ok else " "
            print(f"        {flag}{crit} {c.name}: {c.detail}")
        if r.hints and args.verbose:
            for h in r.hints:
                print(f"        hint: {h}")
        if r.status.value in ("error",):
            worst = max(worst, 2)
        elif r.status.value in ("warn", "offline"):
            worst = max(worst, 1)
    return worst


def cmd_read(args: argparse.Namespace) -> int:
    mgr = SensorManager.from_yaml(args.config, dry_run=args.dry_run)
    try:
        sample = mgr.read(args.id)
    except Exception as e:  # noqa: BLE001
        print(f"read failed: {e}", flush=True)
        return 2
    payload = _jsonable(sample)
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    else:
        for k, v in payload.items():
            print(f"  {k}: {v}")
    return 0


def cmd_kinds(_: argparse.Namespace) -> int:
    load_all_drivers()
    for k in list_registered_kinds():
        print(k)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parent = argparse.ArgumentParser(add_help=False)
    parent.add_argument("-c", "--config", type=Path, default=_default_config(), help="bundle YAML")
    parent.add_argument("--dry-run", action="store_true", help="force dry_run context")

    p = argparse.ArgumentParser(prog="sensors", description="hik-sensors CLI", parents=[parent])
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list", help="list configured devices", parents=[parent])
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("probe", help="read-only probe", parents=[parent])
    sp.add_argument("--id", help="single device id")
    sp.add_argument("--json", action="store_true")
    sp.add_argument("-v", "--verbose", action="store_true")
    sp.set_defaults(func=cmd_probe)

    sp = sub.add_parser("read", help="open → read → close one device", parents=[parent])
    sp.add_argument("--id", required=True, help="device id")
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(func=cmd_read)

    sp = sub.add_parser("kinds", help="list registered driver kinds", parents=[parent])
    sp.set_defaults(func=cmd_kinds)
    return p


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    raise SystemExit(args.func(args))


if __name__ == "__main__":
    main()
