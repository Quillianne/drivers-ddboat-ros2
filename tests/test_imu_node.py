import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def main():
    rclpy.init()
    node = Node('test_imu_node')
    received = False

    def cb(msg):
        nonlocal received
        node.get_logger().info('imu msg received')
        received = True

    node.create_subscription(Imu, 'imu/data_raw', cb, 10)
    while rclpy.ok() and not received:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
