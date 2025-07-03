"""
Publish to `/radio_tx` and subscribe to `/radio_rx` (std_msgs/String) using
rosbridge / roslibpy.  Frames on the wire follow the `id_src:id_dst:length:msg`
format and this script simply deals with the payload text.

Prerequisites
-------------
* rosbridge_server running on port 9090 (see the `rosbridge` service in docker-compose).

Run with:

    python3 test_radio_node.py
"""

import time
import roslibpy
import threading


def main() -> None:
    # Connect to the rosbridge WebSocket (adjust host/port if needed)
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    # Publisher on /radio_tx
    tx_topic = roslibpy.Topic(client, '/radio_tx', 'std_msgs/String')

    # Callback for incoming messages on /radio_rx
    def on_rx(msg):
        print(f"radio rx: {msg['data']}")
        # Clean up in a background thread to avoid joining the current thread
        def _shutdown():
            rx_topic.unsubscribe()
            tx_topic.unadvertise()
            client.terminate()
        threading.Thread(target=_shutdown, daemon=True).start()

    # Subscriber on /radio_rx
    rx_topic = roslibpy.Topic(client, '/radio_rx', 'std_msgs/String')
    rx_topic.subscribe(on_rx)

    # Publish a test message
    tx_msg = {'data': 'ping'}
    tx_topic.publish(roslibpy.Message(tx_msg))

    # Keep the script alive until the callback terminates the client
    try:
        while client.is_connected:
            time.sleep(0.1)
    except KeyboardInterrupt:
        rx_topic.unsubscribe()
        tx_topic.unadvertise()
        client.terminate()


if __name__ == '__main__':
    main()
