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
        ax = msg['linear_acceleration']['x']
        ay = msg['linear_acceleration']['y']
        az = msg['linear_acceleration']['z']
        gx = msg['angular_velocity']['x']
        gy = msg['angular_velocity']['y']
        gz = msg['angular_velocity']['z']
        print("ACC: %d %d %d\tGYR: %d %d %d\r" % (int(ax), int(ay), int(az), int(gx), int(gy), int(gz)))

    def on_mag(msg):
        print("IMU mag: {}".format(msg))


    imu_topic = roslibpy.Topic(
        client,
        '/imu/data_raw',
        'sensor_msgs/Imu'
    )
    imu_topic.subscribe(on_imu)

    mag_topic = roslibpy.Topic(
        client,
        '/imu/mag',
        'geometry_msgs/Vector3Stamped'
    )
    mag_topic.subscribe(on_mag)

    # Keep the script alive until `client.terminate()` is called
    try:
        while client.is_connected:
            time.sleep(0.1)
    except KeyboardInterrupt:
        client.terminate()


if __name__ == '__main__':
    main()
