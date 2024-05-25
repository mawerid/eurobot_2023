import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import String
from std_msgs.msg import Float64
import numpy as np

HEIGHT = 1
WIDTH = 1.5
ROTATION_SPEED = 1.9
LINEAR_SPEED = 0.35
MIN_RADIAL_ANGLE = 0.1


class MoveBase(Node):

    def __init__(self):
        super().__init__('move_base')
        self.wheel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu_topic', self.listener_imu, 10)
        self.collision = self.create_subscription(String, 'obstacle_info', self.anticollision_callback, 10)
        self.aim_subscriber = self.create_subscription(Vector3, 'aim_topic', self.action, 10)
        self.imu_correction = self.create_subscription(Float64, 'imu_correct', self.correction, 10)
        self.planner_talk = self.create_publisher(String, 'mv_base_resp', 10)
        self.robot_place = self.create_subscription(Pose, 'robot_place', self.pose_callback, 10)
        self.scanning = self.create_subscription(String, 'scan_topic', self.scan_callback, 10)
        self.permit = 'move'
        self.imu_info = 0
        self.imu_correct = 0
        self.vel = Twist()
        self.up = HEIGHT
        self.down = -HEIGHT
        self.right = WIDTH
        self.left = -WIDTH
        self.pose_x = 0
        self.pose_y = 0
        self.angle = 0

    def listener_imu(self, msg):
        self.imu_info = float(msg.angular_velocity.z)
        # self.get_logger().info('Imu_information: "%s"' % msg.angular_velocity.z)

    def correction(self, msg):
        self.imu_correct = msg.data

    def anticollision_callback(self, msg):
        self.permit = msg.data

    def pose_callback(self, msg):
        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.angle = Rotation.from_quat(
            np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])).as_euler('zyx',
                                                                                                             degrees=True)[
            0]

    def scan_callback(self, msg):
        if msg.data == 'rotate':
            if self.pose_x > WIDTH / 2:
                self.rotate(dim='overwise')
            else:
                self.rotate(dim='clockwise')
        else:
            self.stop_wheel()

    def rotate(self, dim):
        direction = -1.0
        if dim == 'overwise':
            direction *= direction

        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = ROTATION_SPEED
        self.wheel_publisher.publish(self.vel)

    def go_straight(self, dim, axe):
        direction = -1.0
        if dim == 'backward':
            direction *= direction
        if axe == 'x':
            self.vel.linear.x = LINEAR_SPEED
            self.vel.linear.y = 0.0
            self.vel.angular.z = 0.0
            self.wheel_publisher.publish(self.vel)
        else:
            self.vel.linear.x = 0.0
            self.vel.linear.y = LINEAR_SPEED
            self.vel.angular.z = 0.0
            self.wheel_publisher.publish(self.vel)

    def stop_wheel(self):
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        self.wheel_publisher.publish(self.vel)

    def make_command(self, dif_angle):
        tol = 15
        if 45 >= dif_angle >= -45:
            if dif_angle >= tol:
                self.rotate(dim='clockwise')
            elif dif_angle <= -tol:
                self.rotate(dim='overwise')
            else:
                self.go_straight(dim='forward', axe='x')
        if 135 >= dif_angle > 45:
            if dif_angle >= 90 + tol:
                self.rotate(dim='clockwise')
            elif dif_angle <= 90 - tol:
                self.rotate(dim='overwise')
            self.go_straight(dim='forward', axe='y')
        if (180 >= dif_angle > 135) or (-135 > dif_angle >= -180):
            if (dif_angle <= 180 - tol) and (dif_angle > 0):
                self.rotate(dim='overwise')
            elif (dif_angle >= -180 + tol) and (dif_angle < 0):
                self.rotate(dim='clockwise')
            self.go_straight(dim='backward', axe='x')
        if -45 > dif_angle >= -135:
            if dif_angle > -90 + tol:
                self.rotate(dim='clockwise')
            elif dif_angle < -90 - tol:
                self.rotate(dim='overwise')
            else:
                self.go_straight(dim='backward', axe='y')

    def action(self, msg):
        dif_angle = msg.y + self.angle  # self.imu_info + self.imu_correct
        if self.permit == 'move':
            if msg.x > MIN_RADIAL_ANGLE:
                hside = max(abs(-self.pose_y - self.up), abs(-self.pose_y - self.down))
                vside = max(abs(self.pose_x - self.left), abs(self.right - self.left))
                if (msg.x > hside) and (msg.y >= 0) and (msg.y <= 180):
                    self.make_command(dif_angle=dif_angle - 180)
                elif (msg.x > hside) and (msg.y >= -180) and (msg.y < 0):
                    self.make_command(dif_angle=dif_angle + 180)
                elif (msg.x > vside) and (msg.y >= 90) and (msg.y <= 180):
                    self.make_command(dif_angle=dif_angle - 180)
                elif (msg.x > vside) and (msg.y >= -180) and (msg.y <= -90):
                    self.make_command(dif_angle=dif_angle + 180)
                else:
                    self.make_command(dif_angle=dif_angle)
            else:
                self.stop_wheel()
                resp = String()
                resp.data = "done"
                self.planner_talk.publish(resp)
        else:
            self.stop_wheel()
            resp = String()
            resp.data = "stop_obstacle"
            self.planner_talk.publish(resp)


def main(args=None):
    rclpy.init(args=args)
    move_base = MoveBase()

    rclpy.spin(move_base)

    move_base.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
