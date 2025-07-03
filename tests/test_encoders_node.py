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
        # Exercise services once the first message arrives
        clear_srv = roslibpy.Service(client, '/clear_counts', 'std_srvs/srv/Trigger')
        last_srv = roslibpy.Service(client, '/request_last', 'std_srvs/srv/Trigger')
        try:
            res = clear_srv.call(roslibpy.ServiceRequest(), timeout=5)
            print(f'clear_counts: {res}')
        except Exception as e:
            print(f'clear_counts failed: {e}')
        try:
            res = last_srv.call(roslibpy.ServiceRequest(), timeout=5)
            print(f'request_last: {res}')
        except Exception as e:
            print(f'request_last failed: {e}')

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
