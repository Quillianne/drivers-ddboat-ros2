#!/usr/bin/env python3
"""
End-to-end test for every DDBoat driver via rosbridge / roslibpy.

Run:
    python3 test_all_nodes.py            # assumes rosbridge on localhost:9090
    python3 test_all_nodes.py <host>     # different host, same port 9090
"""

import sys
import time
import threading
from typing import Dict

import roslibpy

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
HOST  = sys.argv[1] if len(sys.argv) > 1 else 'localhost'
PORT  = 9090
TIMEOUT_S = 5.0                      # per-topic timeout

# Sensors we expect to receive at least once
SENSOR_TOPICS: Dict[str, str] = {
    '/fix': 'sensor_msgs/NavSatFix',
    '/encoders': 'std_msgs/Int32MultiArray',
    '/imu/data_raw': 'sensor_msgs/Imu',
    '/temperature_left': 'sensor_msgs/Temperature',
    '/temperature_right': 'sensor_msgs/Temperature',
    '/radio_rx': 'std_msgs/String',
}

# ---------------------------------------------------------------------------
# Test logic
# ---------------------------------------------------------------------------
results: Dict[str, bool] = {t: False for t in SENSOR_TOPICS}

def main() -> None:
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()
    print(f'Connected to rosbridge at ws://{HOST}:{PORT}')

    # ----------------------------------------------------------------------
    # Set up subscriptions
    # ----------------------------------------------------------------------
    subs = []
    for topic, msg_type in SENSOR_TOPICS.items():
        sub = roslibpy.Topic(client, topic, msg_type)
        subs.append(sub)

        def make_cb(name):
            def _cb(_msg):
                results[name] = True
                # Unsubscribe after first hit to reduce traffic
                sub.unsubscribe()
            return _cb

        sub.subscribe(make_cb(topic))

    # ----------------------------------------------------------------------
    # Publish on motors + radio to provoke responses
    # ----------------------------------------------------------------------
    motors_pub = roslibpy.Topic(
        client, '/motors_cmd', 'geometry_msgs/Twist')
    motors_pub.publish(roslibpy.Message({
        'linear':  {'x': 50.0, 'y': 50.0, 'z': 0.0},
        'angular': {'x': 0.0,  'y': 0.0,  'z': 0.0}
    }))
    motors_pub.unadvertise()

    radio_pub = roslibpy.Topic(
        client, '/radio_tx', 'std_msgs/String')
    radio_pub.publish(roslibpy.Message({'data': 'ping'}))
    radio_pub.unadvertise()

    # ----------------------------------------------------------------------
    # Wait for all topics or timeout
    # ----------------------------------------------------------------------
    start = time.time()
    while time.time() - start < TIMEOUT_S and not all(results.values()):
        time.sleep(0.1)

    # ----------------------------------------------------------------------
    # Report
    # ----------------------------------------------------------------------
    print('\n===== TEST SUMMARY =====')
    for topic, ok in results.items():
        print(f'{topic:<20} {"PASS" if ok else "FAIL"}')
    print('========================')

    client.terminate()

    # Exit with non-zero status if any test failed (useful for CI)
    if not all(results.values()):
        missing = [t for t, ok in results.items() if not ok]
        sys.exit(f'No data received on: {missing}')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrupted by user')