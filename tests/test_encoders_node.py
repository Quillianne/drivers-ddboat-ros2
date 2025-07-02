"""
Subscribe to `/encoders` (std_msgs/Int32MultiArray) through rosbridge / roslibpy.

Prerequisites
-------------
* rosbridge_server running on port 9090 (see the `rosbridge` service
  in docker-compose).

Run with:

    python3 test_encoders_node.py
"""

import time
import roslibpy
import threading


def main() -> None:
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    def on_encoders(msg):
        data = msg['data']
        print(f'encoders: {data}')
        # Clean up in a background thread
        def _shutdown():
            enc_topic.unsubscribe()
            client.terminate()
        threading.Thread(target=_shutdown, daemon=True).start()

    enc_topic = roslibpy.Topic(
        client,
        '/encoders',
        'std_msgs/Int32MultiArray'
    )
    enc_topic.subscribe(on_encoders)

    # Keep the script alive until `client.terminate()` is called
    try:
        while client.is_connected:
            time.sleep(0.1)
    except KeyboardInterrupt:
        client.terminate()


if __name__ == '__main__':
    main()
