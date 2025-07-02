"""
Publish a geometry_msgs/Twist to `/motors_cmd` using rosbridge/roslibpy.

Run a rosbridge server (e.g. the `rosbridge` service in docker‑compose on port
9090) and then simply execute this script:

    python3 test_arduino_node.py
"""

import time
import roslibpy
import threading


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
    for i in range(5):
        print(f'Publishing twist #{i+1}: {twist_msg}')
        twist_topic.publish(roslibpy.Message(twist_msg))
        time.sleep(0.1)

    # Clean up from a separate thread
    def _shutdown():
        print('Shutting down: unadvertising and terminating client')
        twist_topic.unadvertise()
        client.terminate()
    threading.Thread(target=_shutdown, daemon=True).start()


if __name__ == '__main__':
    main()
