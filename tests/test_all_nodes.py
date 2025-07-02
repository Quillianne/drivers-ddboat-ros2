#!/usr/bin/env python3
"""
Exercise *all* DDBoat ROS 2 drivers via rosbridge / roslibpy and print
every message received.

Run:
    python3 test_all_nodes.py            # rosbridge on localhost:9090
    python3 test_all_nodes.py 10.0.0.5   # remote rosbridge
"""

import json
import sys
import time
import threading
from textwrap import shorten
from typing import Dict

import roslibpy

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
HOST = sys.argv[1] if len(sys.argv) > 1 else "localhost"
PORT = 9090
TIMEOUT_S = 5.0                      # per-topic timeout

SENSOR_TOPICS: Dict[str, str] = {
    "/fix": "sensor_msgs/NavSatFix",
    "/encoders": "std_msgs/Int32MultiArray",
    "/imu/data_raw": "sensor_msgs/Imu",
    "/temperature_left": "sensor_msgs/Temperature",
    "/temperature_right": "sensor_msgs/Temperature",
    "/radio_rx": "std_msgs/String",
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def pretty(msg) -> str:
    """JSON-dump but truncate long fields so lines stay short."""
    return shorten(json.dumps(msg, separators=(",", ":")), width=120, placeholder="…")


# ---------------------------------------------------------------------------
# Test logic
# ---------------------------------------------------------------------------
results: Dict[str, bool] = {t: False for t in SENSOR_TOPICS}
counts: Dict[str, int] = {t: 0 for t in SENSOR_TOPICS}


def main() -> None:
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()
    print(f"Connected to rosbridge at ws://{HOST}:{PORT}\n")

    # ----------------------------------------------------------------------
    # Subscriptions
    # ----------------------------------------------------------------------
    subs = []

    def make_cb(topic):
        def _cb(msg):
            counts[topic] += 1
            results[topic] = True
            print(f"[{topic}] {pretty(msg)}")
        return _cb

    for t, typ in SENSOR_TOPICS.items():
        sub = roslibpy.Topic(client, t, typ)
        sub.subscribe(make_cb(t))
        subs.append(sub)

    # ----------------------------------------------------------------------
    # Stimulate the system (motors + radio)
    # ----------------------------------------------------------------------
    motors_pub = roslibpy.Topic(client, "/motors_cmd", "geometry_msgs/Twist")
    motors_pub.publish(
        roslibpy.Message(
            {
                "linear": {"x": 50.0, "y": 50.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        )
    )
    motors_pub.unadvertise()

    radio_pub = roslibpy.Topic(client, "/radio_tx", "std_msgs/String")
    radio_pub.publish(roslibpy.Message({"data": "ping"}))
    radio_pub.unadvertise()

    # ----------------------------------------------------------------------
    # Wait
    # ----------------------------------------------------------------------
    start = time.time()
    while time.time() - start < TIMEOUT_S:
        if all(results.values()):
            break
        time.sleep(0.1)

    # ----------------------------------------------------------------------
    # Clean up
    # ----------------------------------------------------------------------
    for sub in subs:
        sub.unsubscribe()
    client.terminate()

    # ----------------------------------------------------------------------
    # Report
    # ----------------------------------------------------------------------
    print("\n===== TEST SUMMARY =====")
    for topic in SENSOR_TOPICS:
        status = "PASS" if results[topic] else "FAIL"
        print(f"{topic:<20} {status}  ({counts[topic]} msg)")
    print("========================")

    if not all(results.values()):
        missing = [t for t, ok in results.items() if not ok]
        sys.exit(f"No data received on: {missing}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user")