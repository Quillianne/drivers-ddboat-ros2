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

    state = {'acc': (0, 0, 0), 'gyro': (0, 0, 0), 'mag': (0, 0, 0)}

    def print_all():
        if state['acc'] and state['gyro'] and state['mag']:
            ax, ay, az = state['acc']
            gx, gy, gz = state['gyro']
            mx, my, mz = state['mag']
            print("MAG: %d %d %d\t ACC: %d %d %d\t GYR: %d %d %d" % (mx, my, mz, ax, ay, az, gx, gy, gz), end='          \r')

    def on_imu(msg):
        ax = int(msg['linear_acceleration']['x'])
        ay = int(msg['linear_acceleration']['y'])
        az = int(msg['linear_acceleration']['z'])
        gx = int(msg['angular_velocity']['x'])
        gy = int(msg['angular_velocity']['y'])
        gz = int(msg['angular_velocity']['z'])
        state['acc'] = (ax, ay, az)
        state['gyro'] = (gx, gy, gz)
        print_all()

    def on_mag(msg):
        mx = int(msg['magnetic_field']['x'])
        my = int(msg['magnetic_field']['y'])
        mz = int(msg['magnetic_field']['z'])
        state['mag'] = (mx, my, mz)
        print_all()


    imu_topic = roslibpy.Topic(
        client,
        '/imu/data_raw',
        'sensor_msgs/Imu'
    )
    imu_topic.subscribe(on_imu)

    mag_topic = roslibpy.Topic(
        client,
        '/imu/mag',
        'sensor_msgs/MagneticField'
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
