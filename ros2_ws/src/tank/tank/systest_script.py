import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Char
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


class Systest_script(Node):

    def __init__(self):
        super().__init__('systest_script')
        self.publisher_stepper = self.create_publisher(Vector3, 'stepper_topic', 10)
        self.wheel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_servo = self.create_publisher(Vector3, 'servo_topic', 10)
        self.publisher_aim = self.create_publisher(Vector3, 'aim_topic', 10)
        self.screen_publisher = self.create_publisher(Char, 'screen_topic', 10)
        self.drum_publisher = self.create_publisher(Vector3, 'stepper_topic', 10)
        self.elevator_publisher = self.create_publisher(Vector3, 'stepper_topic', 10)
        self.servo_publisher = self.create_publisher(Vector3, 'servo_topic', 10)
        self.subscriber_imu = self.create_subscription(Imu, 'imu_topic', self.listener_callback, 10)
        self.subscriptions
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 
        self.vel = Twist()
        self.drum = Vector3()
        self.servo = Vector3()
        self.elevat = Vector3()


    def timer_callback(self):
        if self.i == 0:
            self.get_logger().info('"go right" checking')
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.35
            self.vel.angular.z = 0.0
            self.wheel_publisher.publish(self.vel)
            self.i += 1
        elif self.i == 1:
            self.get_logger().info('"go forward" checking')
            self.vel.linear.x = 0.35
            self.vel.linear.y = 0.0
            self.vel.angular.z = 0.0
            self.wheel_publisher.publish(self.vel)            
            self.i += 1
        elif self.i >= 2 and self.i < 3:
            if self.i >= 2 and self.i < 2.5:
                self.get_logger().info('"rotate clockwise" checking')
                self.vel.linear.x = 0.0
                self.vel.linear.y = 0.0
                self.vel.angular.z = -2.1
                self.wheel_publisher.publish(self.vel)
                self.i += 0.5 
            elif self.i >= 2.5 and self.i < 3:
                self.get_logger().info('"rotate overwise" checking')
                self.vel.linear.x = 0.0
                self.vel.linear.y = 0.0
                self.vel.angular.z = 2.1
                self.wheel_publisher.publish(self.vel)
                self.i += 0.5 
        elif self.i >= 3 and self.i < 4:
            if self.i >= 3 and self.i < 3.5:
                self.get_logger().info('"drum rotate clockwise" checking')
                self.drum.x = 1.0
                self.drum.y = 2.0
                self.drum.z = 120.0
                self.drum_publisher.publish(self.drum)
                self.vel.linear.x = 0.0
                self.vel.linear.y = 0.0
                self.vel.angular.z = 0.0
                self.wheel_publisher.publish(self.vel)
                self.i += 0.5
            elif self.i >= 3.5 and self.i < 4:
                self.get_logger().info('"drum rotate overwise" checking')
                self.drum.x = 1.0
                self.drum.y = 1.0
                self.drum.z = 120.0
                self.drum_publisher.publish(self.drum)
                self.i += 0.5
        elif self.i == 4:
            self.get_logger().info('"servo 1" checking')
            self.servo.x = 1.0
            self.servo.y = 180.0
            self.servo.z = 0.0
            self.servo_publisher.publish(self.servo)
            self.i += 1
        elif self.i == 5:
            self.get_logger().info('"servo 2" checking')
            self.servo.x = 2.0
            self.servo.y = 180.0
            self.servo.z = 0.0
            self.servo_publisher.publish(self.servo)
            self.i += 1
        elif self.i == 6:
            self.get_logger().info('"servo 3" checking')
            self.servo.x = 3.0
            self.servo.y = 180.0
            self.servo.z = 0.0
            self.servo_publisher.publish(self.servo)
            self.i += 1
        elif self.i == 7:
            self.get_logger().info('"servo 4" checking')
            self.servo.x = 4.0
            self.servo.y = 180.0
            self.servo.z = 0.0
            self.servo_publisher.publish(self.servo)
            self.i += 1    
        elif self.i == 8:
            self.get_logger().info('"servo 5" checking')
            self.servo.x = 5.0
            self.servo.y = 180.0
            self.servo.z = 0.0
            self.servo_publisher.publish(self.servo)
            self.i += 1        
        elif self.i == 9:
            self.get_logger().info('"servo 6" checking')
            self.servo.x = 6.0
            self.servo.y = 180.0
            self.servo.z = 0.0
            self.servo_publisher.publish(self.servo)
            self.i += 1
        elif self.i == 10:
            self.get_logger().info('"elevator up" checking')
            self.elevat.x = 2.0
            self.elevat.y = 1.0
            self.elevat.z = 100.0
            self.elevator_publisher.publish(self.elevat)
            self.i += 1
        elif self.i == 11:
            self.get_logger().info('"elevator down" checking')
            self.elevat.x = 2.0
            self.elevat.y = 2.0
            self.elevat.z = 100.0
            self.elevator_publisher.publish(self.elevat)
            self.i += 1
        elif self.i == 12:
            self.get_logger().info('checking done')
            self.i = 0
        
             
    
    def listener_callback(self, msg):
        self.imu_info = float(msg.angular_velocity.z)
        msg = Char()
        msg.data = self.imu_info
        self.screen_publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    systest_script = Systest_script()
    rclpy.spin(systest_script)

    systest_script.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
