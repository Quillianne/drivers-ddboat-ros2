"""
Subscribe to `/fix` (sensor_msgs/NavSatFix) through rosbridge / roslibpy.

Prerequisites
-------------
* rosbridge_server running on port 9090 (see the `rosbridge` service
  in docker‑compose).

Run with:

    python3 test_gps_node.py
"""

import time
import roslibpy
import threading


def main() -> None:
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    def on_fix(msg):
        lat = msg['latitude']
        lon = msg['longitude']
        print("Received fix: {:.6f}, {:.6f}".format(lat, lon))
        # Try a PMTK command service call
        pmtk_srv = roslibpy.Service(client, '/pmtk_cmd', 'ros2_ddboat/srv/PmtkCmd')
        req = roslibpy.ServiceRequest({'command': 'PMTK605'})  # query firmware ver
        try:
            res = pmtk_srv.call(req, timeout=5)
            print('pmtk_cmd response: {}'.format(res.get("response", "")))
        except Exception as e:
            print('pmtk_cmd failed: {}'.format(e))
        # Clean up in a background thread to avoid joining the current thread
        def _shutdown():
            fix_topic.unsubscribe()
            client.terminate()
        threading.Thread(target=_shutdown, daemon=True).start()

    fix_topic = roslibpy.Topic(
        client,
        '/fix',
        'sensor_msgs/NavSatFix'
    )
    fix_topic.subscribe(on_fix)

    # Keep the script alive until `client.terminate()` is called
    try:
        while client.is_connected:
            time.sleep(0.1)
    except KeyboardInterrupt:
        client.terminate()


if __name__ == '__main__':
    main()
