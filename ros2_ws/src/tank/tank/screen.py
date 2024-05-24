import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Char


class Screen(Node):

    def __init__(self):
        super().__init__('screen')
        self.screen_publisher = self.create_publisher(Char, 'display_topic', 10)
        self.screen_listener = self.create_subscription(Vector3, 'screen_info', self.display_pub, 10)
        self.subscriptions

    def display_pub(self, msg):
        disp = Char()
        disp.data = int(msg.x)
        self.screen_publisher.publish(disp)


def main(args=None):
    rclpy.init(args=args)
    screen = Screen()

    rclpy.spin(screen)

    screen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
