"""Print timestamped IMU samples as JSON lines."""
import argparse
from dataclasses import asdict
import json
import sys

from . import DeviceError, IMUClient, ProtocolError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB serial device, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=10, help="Number of samples")
    parser.add_argument("--timeout", type=float, default=1.0, help="Seconds per operation")
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    try:
        with IMUClient(args.port, baudrate=args.baud, timeout=args.timeout) as imu:
            try:
                imu.stop()
                imu.start()
                for _ in range(args.count):
                    print(json.dumps(asdict(imu.read_sample())), flush=True)
            finally:
                try:
                    imu.stop()
                except (OSError, DeviceError, ProtocolError):
                    pass
    except KeyboardInterrupt:
        return 130
    except (OSError, DeviceError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
