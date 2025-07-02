import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


def main():
    rclpy.init()
    node = Node('test_temperature_node')
    received = 0

    def cb(msg):
        nonlocal received
        received += 1
        node.get_logger().info(f'temp {received}: {msg.temperature}')
        if received >= 2:
            rclpy.shutdown()

    node.create_subscription(Temperature, 'temperature_left', cb, 10)
    node.create_subscription(Temperature, 'temperature_right', cb, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
