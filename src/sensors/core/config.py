from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml
from pydantic import BaseModel, Field


class DeviceConfig(BaseModel):
    id: str
    kind: str
    name: str | None = None
    enabled: bool = True
    endpoint: str | None = None
    params: dict[str, Any] = Field(default_factory=dict)
    tags: list[str] = Field(default_factory=list)


class BundleConfig(BaseModel):
    version: int = 1
    site: str = "default"
    dry_run: bool = False
    defaults: dict[str, Any] = Field(default_factory=dict)
    devices: list[DeviceConfig] = Field(default_factory=list)


def _deep_merge(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    out = dict(a)
    for k, v in b.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a mapping: {path}")
    return data


def _try_resolve_recipe_file(raw: str, *, base: Path) -> str | None:
    """Best-effort absolute path for a recipe relative to the sensors YAML dir."""
    name = Path(raw).name
    for cand in (
        Path(raw).expanduser(),
        base / raw,
        base / "rtsi" / name,
        base / "configs" / "rtsi" / name,
        Path.cwd() / raw,
        Path.cwd() / "rtsi" / name,
        Path.cwd() / "configs" / "rtsi" / name,
    ):
        try:
            if cand.is_file():
                return str(cand.resolve())
        except OSError:
            continue
    return None


def _absolutize_rtsi_recipes(devices: list[Any], *, base: Path) -> None:
    """Rewrite relative RTSI recipe paths against the sensors YAML directory."""
    for dev in devices:
        if not isinstance(dev, Mapping):
            continue
        params = dev.get("params")
        if not isinstance(params, dict):
            continue
        for key in ("rtsi_output_recipe", "rtsi_input_recipe"):
            raw = params.get(key)
            if not raw:
                continue
            resolved = _try_resolve_recipe_file(str(raw), base=base)
            if resolved:
                params[key] = resolved


def load_bundle_config(path: str | Path) -> BundleConfig:
    root = Path(path).resolve()
    data = _load_yaml(root)
    includes = data.pop("include", None) or []
    merged: dict[str, Any] = {}
    for rel in includes:
        merged = _deep_merge(merged, _load_yaml((root.parent / rel).resolve()))
    merged = _deep_merge(merged, data)

    devices = merged.get("devices", [])
    if isinstance(devices, Mapping):
        normalized = []
        for did, spec in devices.items():
            item = dict(spec or {})
            item.setdefault("id", did)
            normalized.append(item)
        merged["devices"] = normalized
        devices = merged["devices"]

    if isinstance(devices, list):
        _absolutize_rtsi_recipes(devices, base=root.parent)

    return BundleConfig.model_validate(merged)


def coerce_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(value)
    s = str(value).strip().lower()
    if s in ("0", "false", "no", "off", ""):
        return False
    if s in ("1", "true", "yes", "on"):
        return True
    return default


def device_to_sensor_config(dev: DeviceConfig, kind_defaults: Mapping[str, Any] | None = None) -> dict[str, Any]:
    cfg: dict[str, Any] = dict(kind_defaults or {})
    cfg.update(dev.params)
    cfg["name"] = dev.name or dev.id
    if dev.endpoint is not None:
        cfg["endpoint"] = dev.endpoint
        # Convenience aliases used by serial drivers
        cfg.setdefault("port", dev.endpoint)
        cfg.setdefault("serial", dev.endpoint)
    cfg["tags"] = list(dev.tags)
    cfg["enabled"] = dev.enabled
    return cfg
