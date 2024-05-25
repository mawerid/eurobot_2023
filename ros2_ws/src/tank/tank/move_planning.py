import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
import math
import numpy as np


class MovePlanning(Node):

    def __init__(self):
        super().__init__('move_planning')
        self.command_listener = self.create_subscription(String, 'main_command_topic', self.act_decision, 10)
        self.robot_place = self.create_subscription(Pose, 'robot_place', self.pose_callback, 10)
        self.info_from_mv = self.create_subscription(String, 'mv_base_resp', self.response_callback, 10)
        self.planner_response = self.create_publisher(String, 'main_response_topic', 10)
        self.aim_planner = self.create_publisher(Vector3, 'aim_topic', 10)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.angle = 0.0
        self.aim = "no aim"
        self.task_condition = ""
        self.positions = {
            "left scan place": [-780, 0],
            "right scan place": [780, 0],
            "left base 2": [-1225, 0],
            "right base 2": [1225, 0],
            "right base 1": [1225, 775],
            "left base 3": [-1225, -775],
            "right base 3": [1225, -775],
            "left base 1": [-1225, 775],
            "center": [0, 0]
        }
        self.start_pose = False

    def act_decision(self, msg):
        ans = String()
        match msg.data:
            case "to_left_base_1":
                self.aim = "left base 1"
                self.path_count(aim="left base 1")
                # ans.data = "moving_to_base"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)
            case "to_left_base_2":
                self.aim = "left base 2"
                self.path_count(aim="left base 2")
                # ans.data = "moving_to_base"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)
            case "to_left_base_3":
                self.aim = "left base 3"
                self.path_count(aim="left base 3")
                # ans.data = "moving_to_base"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)
            case "to_right_base_1":
                self.aim = "right base 1"
                self.path_count(aim="right base 1")
                # ans.data = "moving_to_base"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)
            case "to_right_base_2":
                self.aim = "right base 2"
                self.path_count(aim="right base 2")
                # ans.data = "moving_to_base"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)
            case "to_right_base_3":
                self.aim = "right base 3"
                self.path_count(aim="right base 3")
                # ans.data = "moving_to_base"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)
            case "to_left_pot":
                self.aim = "left scan place"
                self.path_count(aim="left scan place")
                # ans.data = "moving_to_pot"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)
            case "to_right_pot":
                self.aim = "right scan place"
                self.path_count(aim="right scan place")
                # ans.data = "moving_to_pot"
                # self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)

            # case "to_unload":
            #   ans.data = "moving_to_unload"
            #   self.get_logger().info('I am "%s"' % self.i)
            #   self.planner_response.publish(ans)

            case _:
                ans.data = "waiting ..."
                self.get_logger().info('I am "%s"' % ans.data)
                # self.planner_response.publish(ans)

    def response_callback(self, msg):
        self.task_condition = msg.data
        resp = String()
        match msg.data:
            case "done":
                resp.data = "I moved to aim"
                self.planner_response.publish(resp)
            case "stop_obstacle":
                resp.data = "Obstacle, I stopped"
                self.planner_response.publish(resp)
            case _:
                resp.data = "Something wrong"
                self.planner_response.publish(resp)

    def pose_callback(self, msg):
        if not self.start_pose:
            ans = String()
            ans.data = "map_done"
            self.planner_response.publish(ans)
            self.start_pose = True

        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.angle = Rotation.from_quat(
            np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])).as_euler('zyx',
                                                                                                             degrees=True)[
            0]

        print(self.pose_x, self.pose_y, self.angle)

    def path_count(self, aim):
        r = math.sqrt((self.positions[aim][1] - self.pose_y) ** 2 +
                      (self.positions[aim][0] - self.pose_x) ** 2)
        phi = math.atan2(self.positions[aim][1] - self.pose_y,
                         self.positions[aim][0] - self.pose_x)
        msg = Vector3()
        msg.x = r
        msg.y = phi * 180 / math.pi
        self.aim_planner.publish(msg)
        print(msg)


def main(args=None):
    rclpy.init(args=args)

    move_planning = MovePlanning()
    rclpy.spin(move_planning)

    move_planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
