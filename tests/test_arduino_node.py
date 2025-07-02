"""
Publish a geometry_msgs/Twist to `/motors_cmd` using rosbridge/roslibpy.

Run a rosbridge server (e.g. the `rosbridge` service in docker‑compose on port
9090) and then simply execute this script:

    python3 test_arduino_node.py
"""

import time
import roslibpy


def main() -> None:
    # Connect to the rosbridge WebSocket (adjust host/port if needed)
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    # Prepare the publisher on /motors_cmd
    twist_topic = roslibpy.Topic(client,
                                 '/motors_cmd',
                                 'geometry_msgs/Twist')

    # Message payload expressed as plain Python dict matching the ROS fields
    twist_msg = {
        'linear':  {'x': 50.0, 'y': 50.0, 'z': 0.0},
        'angular': {'x': 0.0,  'y': 0.0,  'z': 0.0}
    }

    # Publish five times with a short pause
    for _ in range(5):
        twist_topic.publish(roslibpy.Message(twist_msg))
        time.sleep(0.1)

    # Clean up
    twist_topic.unadvertise()
    client.terminate()


if __name__ == '__main__':
    main()
