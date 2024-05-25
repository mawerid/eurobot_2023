import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import String

height = 800
width = 1500


class MainAlgorithm(Node):

    def __init__(self):
        super().__init__('main_algorithm')
        self.response_listener = self.create_subscription(String, 'main_response_topic', self.action, 10)
        self.commander = self.create_publisher(String, 'main_command_topic', 10)
        self.robot_place = self.create_subscription(Pose, 'robot_place', self.pose_callback, 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.command = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.angle = 0.0
        self.plants_on_board = False

    def action(self, msg):
        match msg:
            case "scan_aruco_static_done":
                if self.pose_x > width / 2:
                    com = String()
                    com.data = "to_right_pot"
                    self.command = "to_right_pot"
                    self.commander.publish(com)
                else:
                    com = String()
                    com.data = "to_left_pot"
                    self.command = "to_left_pot"
                    self.commander.publish(com)
            case "I moved to aim":
                self.command = None
                if self.plants_on_board is not True:
                    com = String()
                    com.data = ""
                    # else:
                #   com =
            case None:
                print('I am waiting static markers')

    def timer_callback(self, msg):
        msg = String()
        msg.data = self.command
        self.get_logger().info('Distance "%s"' % self.command_publish)
        self.publisher_aim.publish(msg)

    def pose_callback(self, msg):
        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.angle = Rotation.from_quat(msg.orientation).as_euler('zyx')[0]


def main(args=None):
    rclpy.init(args=args)
    main_algo = MainAlgorithm()

    rclpy.spin(main_algo)

    main_algo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
