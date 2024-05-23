import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import time


class Drum(Node):

    def __init__(self):
        super().__init__('move_base')
        self.drum_publisher = self.create_publisher(Vector3, 'stepper_topic', 10)
        #self.drum_response = self.create_publisher(Bool, 'response_drum', 10)
        self.drum_listener = self.create_subscription(Bool, 'drum_command', self.listener_drum, 10)
        self.drum_listener
        self.drum_response = False
        self.drum = Vector3()

    def listener_drum(self, msg):
        self.drum_response = msg

    def rotate(self, dim):
        if self.drum_response is True:
            self.drum.x = 1.0
            self.drum.y = 1.0
            self.drum.z = 60.0
            self.drum_publisher.publish(self.drum)
            self.drum_response = False


def main(args=None):
    rclpy.init(args=args)
    drum = Drum()

    rclpy.spin(drum)


    drum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()