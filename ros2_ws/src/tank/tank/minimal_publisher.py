import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        #self.publisher_stepper = self.create_publisher(Vector3, 'stepper_topic', 10)
        #self.publisher_servo = self.create_publisher(Vector3, 'servo_topic', 10)
        self.publisher_aim = self.create_publisher(Vector3, 'aim_topic', 10)
        #self.subscriber_imu = self.create_subscription(Imu, 'imu_topic', self.listener_callback, 10)
        #self.subscriptions
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.r = 200.0
        self.phi = 45.0 


    def timer_callback(self):
        msg = Vector3()
        msg.x = self.r
        #if self.r % 2 == 0:
        msg.y = self.phi
        #else:
           # msg.y = -180 + self.phi
        self.get_logger().info('Distance "%s"' % self.r)
        self.publisher_aim.publish(msg)
        self.r -= 1
    
    def listener_callback(self):
        self.get_logger().info(Imu)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    stepper_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
