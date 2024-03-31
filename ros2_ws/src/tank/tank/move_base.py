import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class MoveBase(Node):

    def __init__(self):
        super().__init__('move_base')
        self.wheel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_response = self.create_publisher(Bool, 'response_base', 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu_topic', self.listener_imu, 10)
        self.imu_subscriber
        self.aim_subscriber = self.create_subscription(Vector3, 'aim_topic', self.action, 10)
        self.aim_subscriber 
        self.vel = Twist()

    def listener_imu(self, msg):
        self.imu_info = float(msg.angular_velocity.z)
        #self.get_logger().info('Imu_information: "%s"' % msg.angular_velocity.z)

    def rotate(self, dim):
        if dim == 'clockwise':
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.angular.z = -2.1
            self.wheel_publisher.publish(self.vel)
        else:
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.angular.z = 2.1
            self.wheel_publisher.publish(self.vel)
    
    def go_straight(self, dim, axe):
        if dim == 'forward':
            if axe == 'x':
                self.vel.linear.x = 0.35
                self.vel.linear.y = 0.0
                self.vel.angular.z = 0.0
                self.wheel_publisher.publish(self.vel)
            else:
                self.vel.linear.x = 0.0
                self.vel.linear.y = 0.35
                self.vel.angular.z = 0.0
                self.wheel_publisher.publish(self.vel)
        else:
            if axe == 'x':
                self.vel.linear.x = -0.35
                self.vel.linear.y = 0.0
                self.vel.angular.z = 0.0
                self.wheel_publisher.publish(self.vel)
            else:
                self.vel.linear.x = 0.0
                self.vel.linear.y = -0.35
                self.vel.angular.z = 0.0
                self.wheel_publisher.publish(self.vel)

    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.wheel_publisher.publish(self.vel)

    def action(self, msg):
        tol = 25
        dim = 0.5
        if (msg.y <= 45 and msg.y >= -45):
            if(self.imu_info >= tol):
                self.rotate(dim = 'clockwise')
            elif(self.imu_info <= -tol):
               self.rotate(dim = 'overwise')
            else:    
                self.go_straight(dim = 'forward', axe = 'x') 
        if (msg.y <= 135 and msg.y > 45):
            if(self.imu_info >= 90+tol):
                self.rotate(dim = 'clockwise')
            elif(self.imu_info <= 90-tol):
                self.rotate(dim = 'overwise')
            self.go_straight(dim = 'forward', axe = 'y')               
        if ((msg.y <= 180 and msg.y > 135) or (msg.y < -135 and msg.y >= -180)):
            if(self.imu_info >= 180 - tol):
                self.rotate(dim = 'overwise')
            elif(self.imu_info <= -180 + tol):
                self.rotate(dim = 'clockwise')
            self.go_straight(dim = 'backward', axe = 'x') 
        if (msg.y < -45 and msg.y >= -135):
            if(self.imu_info > -45 + tol):
                self.rotate(dim = 'clockwise')
            elif(self.imu_info < -45 - tol):
                self.rotate(dim = 'overwise')
            else:
                self.go_straight(dim = 'backward', axe = 'y')                
        #if msg.x <= dim:
         #   resp = Bool()
          #  resp = True
           # self.move_response.publish(resp)
           # self.stop()
        #else:
         #   resp = Bool()
          #  resp = False
           # self.move_response.publish(resp)


def main(args=None):
    rclpy.init(args=args)
    move_base = MoveBase()

    rclpy.spin(move_base)


    move_base.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()