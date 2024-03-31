import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
#from sensor_msgs.msg import 

class Planer(Node):

    def __init__(self):
        super().__init__('action_plan')
        #self = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.move_response = self.create_publisher(Bool, 'response_stat', 10)
        self.eye_cam_subscriber = self.create_subscription(Imu, 'imu_topic', self.listener_imu, 10)
        self.eye_cam_subscriber
        self._subscriber = self.create_subscription(Vector3, 'aim_topic', self.action, 10)
        self.aim_subscriber 
        

def main(args=None):
    rclpy.init(args=args)
    action_planner = Planer()

    rclpy.spin(action_planner)

    action_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()