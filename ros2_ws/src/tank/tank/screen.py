import rclpy
from rclpy.node import Node

from std_msgs.msg import Char

class Screen(Node):

    def __init__(self):
        super().__init__('screen')
        self.screen_publisher = self.create_publisher(Char, 'screen_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0        
    
    def timer_callback(self):
        msg = Char()
        msg.data = self.i
        self.get_logger().info('Points "%s"' % self.i)
        self.screen_publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    screen = Screen()

    rclpy.spin(screen)

    screen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()