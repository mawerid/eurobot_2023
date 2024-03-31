import rclpy
from rclpy.node import Node

#from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import time

class Drum(Node):

    def __init__(self):
        super().__init__('move_base')
        self.drum_publisher = self.create_publisher(Vector3, 'stepper_topic', 10)
        self.drum_response = self.create_publisher(Bool, 'response_drum', 10)
        '''
        self.imu_subscriber = self.create_subscription(Imu, 'imu_topic', self.listener_imu, 10)
        self.imu_subscriber
        self.aim_subscriber = self.create_subscription(Vector3, 'aim_topic', self.action, 10)
        self.aim_subscriber 
        self.vel = Vector3()

    def listener_imu(self, msg):
        self.imu_info = msg.angular_velocity
        self.get_logger().info('Imu_information: "%s"' % msg.angular_velocity)

    def rotate(self, dim):
        if dim == 'clockwise':
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.angular.z = 2.15
            self.wheel_publisher.publish(self.vel)
        else:
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.angular.z = -2.15
            self.wheel_publisher.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    drum = Drum()

    rclpy.spin(drum)


    drum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()