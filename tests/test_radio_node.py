import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('test_radio_node')
    sub_received = False

    def rx_cb(msg):
        nonlocal sub_received
        node.get_logger().info(f'radio rx: {msg.data}')
        sub_received = True

    pub = node.create_publisher(String, 'radio_tx', 10)
    node.create_subscription(String, 'radio_rx', rx_cb, 10)
    msg = String()
    msg.data = 'ping'
    pub.publish(msg)
    while rclpy.ok() and not sub_received:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
