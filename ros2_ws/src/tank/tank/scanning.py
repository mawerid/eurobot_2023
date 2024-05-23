import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.command_listener = self.create_subscription(String, 'main_command_topic', self.act_decision, 10)
        self.scanner_response = self.create_publisher(String, 'main_response_topic', 10)
        self.publisher_aim = self.create_publisher(Vector3, 'aim_topic', 10)
        self.subscriber_imu = self.create_subscription(Imu, 'imu_topic', self.listener_callback, 10)
        self.subscriptions

    def listener_callback(self):
        self.get_logger().info(Imu)

    def act_decision(self):
        print('ss')


def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()
    rclpy.spin(scanner)

    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
