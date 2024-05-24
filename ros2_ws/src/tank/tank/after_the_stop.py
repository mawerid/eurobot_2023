import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from std_msgs.msg import String


# система антислокновления
class AfterStop(Node):
    def __init__(self):
        super().__init__('after_the_stop')
        self.command_listener = self.create_subscription(String, 'main_command_topic', self.act_decision, 10)
        self.static_aruco_sub = self.create_subscription(PoseStamped, 'static_aruco', self.static_aruco_info, 10)
        self.publisher_aim = self.create_publisher(Vector3, 'aim_topic', 10)
        self.robot_place = self.create_subscription(Pose, 'robot_place', self.pose_callback, 10)
        self.enemy_place = self.create_subscription(Pose, 'enemy_place', self.enemy_callback, 10)
        self.marker_20 = [0., 0., 0.]
        self.marker_21 = [0., 0., 0.]
        self.marker_22 = [0., 0., 0.]
        self.marker_23 = [0., 0., 0.]
        self.our_robot = [0., 0., 0.]
        self.enemy_robot = [0., 0., 0.]

    # принимаю координаты маркеров
    def static_aruco_info(self, msg):
        if msg.x == 20:
            self.marker_20[0] = msg.x
            self.marker_20[1] = msg.y
            self.marker_20[2] = msg.z
        if msg.x == 21:
            self.marker_21[0] = msg.x
            self.marker_21[1] = msg.y
            self.marker_21[2] = msg.z
        if msg.x == 22:
            self.marker_22[0] = msg.x
            self.marker_22[1] = msg.y
            self.marker_22[2] = msg.z
        if msg.x == 23:
            self.marker_23[0] = msg.x
            self.marker_23[1] = msg.y
            self.marker_23[2] = msg.z

    # принимаю координаты робота и enemy
    def pose_callback(self, msg):
        self.our_robot[0] = 2
        self.our_robot[1] = msg.x
        self.our_robot[2] = msg.y

    def enemy_callback(self, msg):
        self.enemy_robot[0] = 1
        self.enemy_robot[1] = msg.x
        self.enemy_robot[2] = msg.y

    def act_decision(self, msg):
        if msg.data == "Obstacle, I stopped":
            rad = 300.
            aim = Vector3()
            aim.x = rad
            k_norm = (self.marker_20[2] - self.marker_22[2]) / 980
            up = self.marker_22[2] - k_norm * 510
            down = self.marker_20[2] + k_norm * 510
            left = self.marker_21[1] - k_norm * 750
            right = self.marker_20[1] + k_norm * 750

            x_middle = (left + right) / 2
            y_middle = (up + down) / 2

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

            if self.our_robot[1] >= x_middle:  # 1 и 4 четверти

                if self.our_robot[2] >= y_middle:  # условие, что робот в 1 четверти

                    if self.our_robot[1] > self.enemy_robot[1]:  # условие, что наш робот справа
                        # 1 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            if (abs(self.our_robot[1] - right)) > (abs(self.our_robot[2] - up)):
                                print(315)
                                aim.y = -45
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - right)) < (abs(self.our_robot[2] - up)):
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

                if self.our_robot[2] < y_middle:  # условие, что робот в 4 четверти

                    if self.our_robot[1] > self.enemy_robot[1]:  # условие, что наш робот справа
                        # 1 ВАРИАНТ
                        if self.our_robot[2] > self.enemy_robot[2]:  # условие, что наш робот выше
                            print(135)
                            aim.y = 135
                            self.publisher_aim.publish(aim)
                        # 2 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            if (abs(self.our_robot[1] - right)) > (abs(self.our_robot[2] - down)):
                                print(45)
                                aim.y = 45
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - right)) < (abs(self.our_robot[2] - down)):
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

            if self.our_robot[1] < x_middle:  # 2 и 3 четверти

                if self.our_robot[2] >= y_middle:  # условие, что робот в 2 четверти

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
                            if (abs(self.our_robot[1] - left)) > (abs(self.our_robot[2] - up)):
                                print(225)
                                aim.y = -135
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - left)) < (abs(self.our_robot[2] - up)):
                                print(45)
                                aim.y = 45
                                self.publisher_aim.publish(aim)
                        # 4 ВАРИАНТ
                        if self.our_robot[2] < self.enemy_robot[2]:  # условие, что наш робот ниже
                            print(315)
                            aim.y = -45
                            self.publisher_aim.publish(aim)

                if self.our_robot[2] < y_middle:  # условие, что робот в 3 четверти

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
                            if (abs(self.our_robot[1] - left)) > (abs(self.our_robot[2] - down)):
                                print(135)
                                aim.y = 135
                                self.publisher_aim.publish(aim)
                            if (abs(self.our_robot[1] - left)) < (abs(self.our_robot[2] - down)):
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
