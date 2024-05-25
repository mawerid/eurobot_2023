import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import numpy as np

HEIGHT = 1.0
WIDTH = 1.5

TIMER_PERIOD = 0.2


class MainAlgorithm(Node):

    def __init__(self):
        super().__init__('main_algorithm')
        self.response_listener = self.create_subscription(String, 'main_response_topic', self.action, 10)
        self.commander = self.create_publisher(String, 'main_command_topic', 10)
        self.robot_place = self.create_subscription(Pose, 'robot_place', self.pose_callback, 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.command = ""
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.angle = 0.0
        self.plants_on_board = False

    def action(self, msg):
        match msg.data:
            case "map_done":
                # if self.pose_x > width / 2:
                #     com = String()
                #     com.data = "to_right_pot"
                #     self.command = "to_right_pot"
                #     self.commander.publish(com)
                # else:
                #     com = String()
                #     com.data = "to_left_pot"
                #     self.command = "to_left_pot"
                #     self.commander.publish(com)
                com = String()
                com.data = "to_left_base_1"
                self.command = "to_left_base_1"
                self.commander.publish(com)
            # case "I moved to aim":
            # self.command = None
            # if self.plants_on_board is not True:
            #   com = String()
            #  com.data = ""
            # else:
            #   com =
            # case None:
            # print('I am waiting static markers')

    def timer_callback(self):
        msg = String()
        msg.data = self.command
        # self.get_logger().info('Distance "%s"' % self.command)
        self.commander.publish(msg)

    def pose_callback(self, msg):
        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.angle = Rotation.from_quat(
            np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])).as_euler('zyx',
                                                                                                             degrees=True)[
            0]


def main(args=None):
    rclpy.init(args=args)
    main_algo = MainAlgorithm()

    rclpy.spin(main_algo)

    main_algo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
