import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.msg import OccupancyGrid

from nav2_core.path_follower import EuclideanFollower
from nav2_costmap2d import Costmap2D


class MyPathPlanner(Node):

    def __init__(self):
        super().__init__('my_path_planner')

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odometry_callback, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.costmap = Costmap2D(self)
        self.path_planner = PathPlanner(self.costmap)
        self.path_follower = EuclideanFollower(self.costmap, with_time=True)

        self.current_pose = None
        self.target_pose = None
        self.occupancy_grid = None
        self.path = None

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose

    def target_pose_callback(self, msg):
        self.target_pose = msg.pose

    def map_callback(self, msg):
        self.occupancy_grid = msg

    def plan(self):
        if self.current_pose is None or self.target_pose is None or self.occupancy_grid is None:
            self.get_logger().warn("Waiting for odometry, target pose, or map")
            return

        self.costmap.update_map(self.occupancy_grid)

        plan = self.path_planner.plan(self.current_pole, self.target_pose)

        if plan.waypoints:
            self.path = plan.waypoints
        else:
            self.get_logger().warn("Path planning failed!")

    def follow_path(self):
        if self.path is None or self.current_pose is None:
            return

        twist_cmd = self.path_follower.get_velocity_command(self.current_pose, self.path[0])
        self.cmd_vel_pub.publish(twist_cmd)

    def run(self):
        while rclpy.ok():
            self.plan()
            if self.path:
                self.follow_path()
            rclpy.spin_once(self)
            rclpy.sleep(1.0)


def main():
    rclpy.init_occupation()
    node = MyPathPlanner()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
