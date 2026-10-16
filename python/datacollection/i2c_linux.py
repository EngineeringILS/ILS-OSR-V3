"""Linux I2C adapter discovery for the ESP32 i2c-tiny-usb bridge."""

from __future__ import annotations

from pathlib import Path
from typing import Optional


DEFAULT_SYSFS_I2C_ROOT = Path("/sys/class/i2c-dev")
I2C_TINY_USB_NAME = "i2c-tiny-usb"
DEFAULT_I2C_FREQUENCY_HZ = 400_000


def find_i2c_adapter(
    adapter_name: str = I2C_TINY_USB_NAME,
    *,
    sysfs_root: Path = DEFAULT_SYSFS_I2C_ROOT,
) -> int:
    """Return the unique Linux bus ID whose sysfs name contains ``adapter_name``."""

    matches: list[int] = []
    for adapter in sorted(Path(sysfs_root).glob("i2c-*")):
        try:
            bus_id = int(adapter.name.removeprefix("i2c-"))
            name = (adapter / "name").read_text(encoding="utf-8").strip()
        except (OSError, ValueError):
            continue
        if adapter_name.casefold() in name.casefold():
            matches.append(bus_id)

    if not matches:
        raise RuntimeError(
            f"No {adapter_name!r} adapter found under {sysfs_root}; "
            "bind the Linux i2c-tiny-usb driver before starting motion control"
        )
    if len(matches) > 1:
        buses = ", ".join(f"/dev/i2c-{bus_id}" for bus_id in matches)
        raise RuntimeError(
            f"Multiple {adapter_name!r} adapters found ({buses}); set I2C_BUS_ID explicitly"
        )
    return matches[0]


def create_i2c_tiny_usb_bus(
    bus_id: Optional[int] = None,
    *,
    frequency: int = DEFAULT_I2C_FREQUENCY_HZ,
) -> object:
    """Open the bridge as a CircuitPython-compatible ExtendedI2C object."""

    if frequency <= 0:
        raise ValueError("frequency must be greater than zero")
    if bus_id is None:
        bus_id = find_i2c_adapter()

    from adafruit_extended_bus import ExtendedI2C

    return ExtendedI2C(bus_id, frequency=frequency)
