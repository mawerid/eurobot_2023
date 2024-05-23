import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class MainAlgorithm(Node):

    def __init__(self):
        super().__init__('main_algorithm')
        self.response_listener = self.create_subscription(String, 'main_response_topic', self.action, 10)
        self.commander = self.create_publisher(String, 'main_command_topic', 10)  
        self.subscriptions 
        
           


def main(args=None):
    rclpy.init(args=args)
    main_algo = MainAlgorithm()

    rclpy.spin(main_algo)

    main_algo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()