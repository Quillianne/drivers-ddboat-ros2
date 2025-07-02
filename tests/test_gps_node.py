import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


def main():
    rclpy.init()
    node = Node('test_gps_node')
    received = False

    def cb(msg):
        nonlocal received
        node.get_logger().info(f"fix: {msg.latitude} {msg.longitude}")
        print(f"fix: {msg.latitude} {msg.longitude}")
        received = True

    node.create_subscription(NavSatFix, 'fix', cb, 10)
    while rclpy.ok() and not received:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
