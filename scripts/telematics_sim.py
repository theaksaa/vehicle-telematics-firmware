#!/usr/bin/env python3
"""Drive the firmware with physical CAN frames and simulated GNSS fixes."""

import argparse
import math
import signal
import threading
import time

import can
import serial


def parse_args():
    parser = argparse.ArgumentParser(
        description="Simulate a moving vehicle through SocketCAN and the shell UART"
    )
    parser.add_argument("--serial", required=True, help="board shell port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--can", default="can0", help="SocketCAN interface")
    parser.add_argument("--duration", type=float, default=90.0, help="seconds; 0 runs forever")
    parser.add_argument("--rate", type=float, default=10.0, help="updates per second")
    parser.add_argument("--latitude", type=float, default=44.8125)
    parser.add_argument("--longitude", type=float, default=20.4612)
    parser.add_argument("--altitude", type=float, default=117.0)
    return parser.parse_args()


def drive_state(elapsed):
    cycle = elapsed % 60.0
    if cycle < 10.0:
        speed = cycle * 5.0
        acceleration = 5.0
    elif cycle < 35.0:
        speed = 50.0 + 4.0 * math.sin((cycle - 10.0) * 0.3)
        acceleration = 1.2 * math.cos((cycle - 10.0) * 0.3)
    elif cycle < 45.0:
        speed = 50.0 + (cycle - 35.0) * 3.0
        acceleration = 3.0
    elif cycle < 52.0:
        speed = 80.0
        acceleration = 0.0
    else:
        speed = max(0.0, 80.0 - (cycle - 52.0) * 10.0)
        acceleration = -10.0

    throttle = max(0.0, min(100.0, 14.0 + acceleration * 5.0))
    rpm = 800.0 if speed < 0.5 else 1050.0 + speed * 30.0 + throttle * 5.0
    bearing = (35.0 + elapsed * 2.0) % 360.0
    return speed, rpm, throttle, bearing


def can_frames(speed_kph, rpm, throttle_pct):
    speed_raw = max(0, min(0x7FFF, round(speed_kph / 0.01)))
    speed_payload = (speed_raw << 17).to_bytes(8, byteorder="little")

    powertrain = bytearray(8)
    rpm_raw = max(0, min(0xFFFF, round(rpm / 0.25)))
    throttle_raw = max(0, min(0xFF, round(throttle_pct / 0.4)))
    powertrain[2:4] = rpm_raw.to_bytes(2, byteorder="little")
    powertrain[5] = throttle_raw

    return (
        can.Message(arbitration_id=0x1A0, data=speed_payload, is_extended_id=False),
        can.Message(arbitration_id=0x280, data=powertrain, is_extended_id=False),
    )


def shell_command(port, command):
    port.write((command + "\r").encode("ascii"))
    port.flush()


def serial_reader(port, stopped):
    while not stopped.is_set():
        try:
            data = port.read(port.in_waiting or 1)
        except serial.SerialException:
            stopped.set()
            return
        if data:
            print(data.decode("utf-8", errors="replace"), end="", flush=True)


def main():
    args = parse_args()
    if args.rate <= 0.0 or args.duration < 0.0:
        raise SystemExit("--rate must be positive and --duration cannot be negative")

    stopped = threading.Event()

    def request_stop(_signum=None, _frame=None):
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with serial.Serial(args.serial, args.baud, timeout=0.1) as console, can.Bus(
        interface="socketcan", channel=args.can
    ) as bus:
        reader = threading.Thread(target=serial_reader, args=(console, stopped), daemon=True)
        reader.start()
        shell_command(console, "sim start")
        time.sleep(0.25)

        latitude = args.latitude
        longitude = args.longitude
        start = time.monotonic()
        previous = start
        next_update = start
        last_report_second = -1

        try:
            while not stopped.is_set():
                now = time.monotonic()
                elapsed = now - start
                if args.duration and elapsed >= args.duration:
                    break

                speed, rpm, throttle, bearing = drive_state(elapsed)
                dt = now - previous
                previous = now
                distance_m = speed / 3.6 * dt
                heading = math.radians(bearing)
                latitude += (distance_m * math.cos(heading)) / 111_320.0
                longitude += (distance_m * math.sin(heading)) / (
                    111_320.0 * math.cos(math.radians(latitude))
                )

                for frame in can_frames(speed, rpm, throttle):
                    bus.send(frame, timeout=0.1)

                shell_command(
                    console,
                    f"sim gps {latitude:.9f} {longitude:.9f} {args.altitude:.1f} "
                    f"{speed:.2f} {bearing:.2f} 10",
                )

                report_second = int(elapsed)
                if report_second != last_report_second:
                    last_report_second = report_second
                    shell_command(console, "vehicle status")
                    shell_command(console, "gps fix")

                next_update += 1.0 / args.rate
                stopped.wait(max(0.0, next_update - time.monotonic()))
        finally:
            shell_command(console, "sim stop")
            time.sleep(0.2)
            stopped.set()
            reader.join(timeout=0.5)


if __name__ == "__main__":
    main()
