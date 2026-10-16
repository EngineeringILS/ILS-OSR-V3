"""Print LSM9DS1 samples as CSV until interrupted."""

import os
import time

try:
    from .data_methods import LSM9DS1Collector, create_lsm9ds1_i2c
    from .data_types import LSM9DS1Data
    from .i2c_linux import create_i2c_tiny_usb_bus
except ImportError:  # Allow ``python3 main.py`` from this directory.
    from data_methods import LSM9DS1Collector, create_lsm9ds1_i2c
    from data_types import LSM9DS1Data
    from i2c_linux import create_i2c_tiny_usb_bus


def main() -> None:
    configured_bus_id = os.environ.get("I2C_BUS_ID")
    bus_id = int(configured_bus_id) if configured_bus_id is not None else None
    i2c_bus = create_i2c_tiny_usb_bus(bus_id, frequency=400_000)
    collector = LSM9DS1Collector(create_lsm9ds1_i2c(i2c_bus), sample_hz=20.0)
    last_sequence = -1
    print(LSM9DS1Data.csv_header())
    with collector:
        while True:
            sample = collector.latest(max_age_seconds=0.25)
            if sample is not None and sample.sequence != last_sequence:
                print(sample.to_csv_row(), flush=True)
                last_sequence = sample.sequence
            time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
