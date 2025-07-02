import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = Node('test_arduino_node')
    pub = node.create_publisher(Twist, 'motors_cmd', 10)
    msg = Twist()
    msg.linear.x = 50.0
    msg.linear.y = 50.0
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
