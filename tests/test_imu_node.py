"""
Subscribe to `/imu/data_raw` (sensor_msgs/Imu) through rosbridge / roslibpy.

Prerequisites
-------------
* rosbridge_server running on port 9090 (see the `rosbridge` service
  in docker-compose).

Run with:

    python3 test_imu_node.py
"""

import time
import roslibpy
import threading


def main() -> None:
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    def on_imu(msg):
        print(f"IMU msg received: {msg}")

        calib_srv = roslibpy.Service(client, '/fast_heading_calibration', 'std_srvs/srv/Trigger')
        try:
            res = calib_srv.call(roslibpy.ServiceRequest(), timeout=5)
            print(f'fast_heading_calibration: {res}')
        except Exception as e:
            print(f'fast_heading_calibration failed: {e}')

        # Clean up in a background thread
        def _shutdown():
            imu_topic.unsubscribe()
            client.terminate()

        threading.Thread(target=_shutdown, daemon=True).start()

    imu_topic = roslibpy.Topic(
        client,
        '/imu/data_raw',
        'sensor_msgs/Imu'
    )
    imu_topic.subscribe(on_imu)

    # Keep the script alive until `client.terminate()` is called
    try:
        while client.is_connected:
            time.sleep(0.1)
    except KeyboardInterrupt:
        client.terminate()


if __name__ == '__main__':
    main()
