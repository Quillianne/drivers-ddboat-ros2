"""
Subscribe to `/temperature_left` and `/temperature_right` (sensor_msgs/Temperature)
through rosbridge / roslibpy.

Prerequisites
-------------
* rosbridge_server running on port 9090 (see the `rosbridge` service
  in docker-compose).

Run with:

    python3 test_temp_node.py
"""

import time
import roslibpy


def main() -> None:
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    received = {'count': 0}

    def on_temp(msg):
        received['count'] += 1
        temp = msg['temperature']
        print(f'temp {received["count"]}: {temp}')
        if received['count'] >= 2:
            left_topic.unsubscribe()
            right_topic.unsubscribe()
            client.terminate()

    left_topic = roslibpy.Topic(
        client,
        '/temperature_left',
        'sensor_msgs/Temperature'
    )
    right_topic = roslibpy.Topic(
        client,
        '/temperature_right',
        'sensor_msgs/Temperature'
    )
    left_topic.subscribe(on_temp)
    right_topic.subscribe(on_temp)

    try:
        while client.is_connected:
            time.sleep(0.1)
    except KeyboardInterrupt:
        left_topic.unsubscribe()
        right_topic.unsubscribe()
        client.terminate()


if __name__ == '__main__':
    main()
