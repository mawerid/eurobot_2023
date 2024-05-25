import numpy as np
import cv2
import rclpy
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import String


class AfterStop(Node):
    def __init__(self):
        super().__init__('after_the_stop')
        self.command_listener = self.create_subscription(String, 'main_command_topic', self.act_decision, 10)
        self.publisher_aim = self.create_publisher(Vector3, 'aim_topic', 10)
        self.robot_place = self.create_subscription(Pose, 'robot_place', self.pose_callback, 10)
        self.enemy_place = self.create_subscription(Pose, 'enemy_place', self.enemy_callback, 10)
        self.our_robot = [0., 0., 0., 0.]  # заполняем позже
        self.enemy_robot = [0., 0., 0., 0.]  # заполняем позже
        self.center = [0., 0.]
        self.up = 1.
        self.down = -1.
        self.left = -1.5
        self.right = 1.5

    # принимаю координаты робота и enemy
    def pose_callback(self, msg):
        self.our_robot[0] = 2
        self.our_robot[1] = msg.position.x
        self.our_robot[2] = msg.position.y
        self.our_robot[3] = Rotation.from_quat(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])).as_euler('zyx', degrees=True)[0]

    def enemy_callback(self, msg):
        self.enemy_robot[0] = 1
        self.enemy_robot[1] = msg.position.x
        self.enemy_robot[2] = msg.position.y
        self.enemy_robot[3] = Rotation.from_quat(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])).as_euler('zyx', degrees=True)[0]

    def act_decision(self, msg):
        if msg.data == "Obstacle, I stopped":
            rad = 300.
            aim = Vector3()
            aim.x = rad

            # 1 вариант
            # [[0, OUR],
            #  [ENEMY, 0]]
            #
            # 2 вариант
            # [[ENEMY, 0],
            #  [0, OUR]]
            #
            # 3 вариант
            # [[OUR, 0],
            #  [0, ENEMY]]
            #
            # 4 вариант
            # [[0, ENEMY],
            #  [OUR, 0]]

            if self.our_robot[1] >= self.center[0]:  # 1 и 4 четверти

                if self.our_robot[2] >= self.center[1]:  # условие, что робот в 1 четверти

                    if self.our_robot[1] > self.enemy_robot[1]:  # условие, что наш робот справа
                        # 1 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            if (abs(self.our_robot[1] - self.right)) > (abs(self.our_robot[2] - self.up)):
                                print(315)
                                aim.y = -45
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - self.right)) < (abs(self.our_robot[2] - self.up)):
                                print(135)
                                aim.y = 135
                                self.publisher_aim.publish(aim)
                        # 2 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            print(225)
                            aim.y = -135
                            self.publisher_aim.publish(aim)

                    if self.our_robot[1] < self.enemy_robot[1]:  # условие, что наш робот слева
                        # 3 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            print(225)
                            aim.y = -135
                            self.publisher_aim.publish(aim)
                        # 4 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            print(225)
                            aim.y = -135
                            self.publisher_aim.publish(aim)

                if self.our_robot[2] < self.center[1]:  # условие, что робот в 4 четверти

                    if self.our_robot[1] > self.enemy_robot[1]:  # условие, что наш робот справа
                        # 1 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            print(135)
                            aim.y = 135
                            self.publisher_aim.publish(aim)
                        # 2 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            if (abs(self.our_robot[1] - self.right)) > (abs(self.our_robot[2] - self.down)):
                                print(45)
                                aim.y = 45
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - self.right)) < (abs(self.our_robot[2] - self.down)):
                                print(225)
                                aim.y = -135
                                self.publisher_aim.publish(aim)

                    if self.our_robot[1] < self.enemy_robot[1]:  # условие, что наш робот слева
                        # 3 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            print(135)
                            aim.y = 135
                            self.publisher_aim.publish(aim)
                        # 4 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            print(135)
                            aim.y = 135
                            self.publisher_aim.publish(aim)

            if self.our_robot[1] < self.center[0]:  # 2 и 3 четверти

                if self.our_robot[2] >= self.center[1]:  # условие, что робот в 2 четверти

                    if self.our_robot[1] > self.enemy_robot[1]:  # условие, что наш робот справа
                        # 1 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            print(315)
                            aim.y = -45
                            self.publisher_aim.publish(aim)
                        # 2 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            print(315)
                            aim.y = -45
                            self.publisher_aim.publish(aim)

                    if self.our_robot[1] < self.enemy_robot[1]:  # условие, что наш робот слева
                        # 3 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            if (abs(self.our_robot[1] - self.left)) > (abs(self.our_robot[2] - self.up)):
                                print(225)
                                aim.y = -135
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - self.left)) < (abs(self.our_robot[2] - self.up)):
                                print(45)
                                aim.y = 45
                                self.publisher_aim.publish(aim)
                        # 4 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            print(315)
                            aim.y = -45
                            self.publisher_aim.publish(aim)

                if self.our_robot[2] < self.center[1]:  # условие, что робот в 3 четверти

                    if self.our_robot[1] > self.enemy_robot[1]:  # условие, что наш робот справа
                        # 1 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            print(45)
                            aim.y = 45
                            self.publisher_aim.publish(aim)
                        # 2 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            print(45)
                            aim.y = 45
                            self.publisher_aim.publish(aim)

                    if self.our_robot[1] < self.enemy_robot[1]:  # условие, что наш робот слева
                        # 3 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            print(45)
                            aim.y = 45
                            self.publisher_aim.publish(aim)
                        # 4 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            if (abs(self.our_robot[1] - self.left)) > (abs(self.our_robot[2] - self.down)):
                                print(135)
                                aim.y = 135
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - self.left)) < (abs(self.our_robot[2] - self.down)):
                                print(315)
                                aim.y = -45
                                self.publisher_aim.publish(aim)


def main(args=None):
    rclpy.init(args=args)
    after_the_stop = AfterStop()

    rclpy.spin(after_the_stop)

    after_the_stop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
