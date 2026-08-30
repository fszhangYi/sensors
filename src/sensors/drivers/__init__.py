from __future__ import annotations

"""Import all drivers so @register_sensor side-effects run."""


def load_all_drivers() -> None:
    from sensors.drivers.arm import follower as _arm  # noqa: F401
    from sensors.drivers.bus import serial_bus as _bus  # noqa: F401
    from sensors.drivers.camera import realsense as _rs  # noqa: F401
    from sensors.drivers.extensions import stubs as _ext  # noqa: F401
    from sensors.drivers.gello import leader as _gello  # noqa: F401
    from sensors.drivers.gripper import dh_ag95 as _grip  # noqa: F401
    from sensors.drivers.pipeline import collect as _pipe  # noqa: F401
