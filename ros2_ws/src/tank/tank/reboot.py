import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from time import sleep
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# Сисема нумерации пинов на плате:
# по номеру пина на гребенке коннектора GPIO, например: 26 (IO7), 40 (IO21)
GPIO.setmode(GPIO.BOARD)
GPIO_PICO_1 = 29
GPIO_PICO_2 = 31
GPIO.setup(GPIO_PICO_1, GPIO.OUT)
GPIO.setup(GPIO_PICO_2, GPIO.OUT)


class Reboot(Node):

    def __init__(self):
        super().__init__('reboot')
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pico1_topic = '/cmd_vel'
        self.pico2_topic = '/pico2_status'
        self.imu_subscriber = self.create_subscription(Imu, 'imu_topic', self.listener_imu, 10)
        self.imu_correct = self.create_publisher(Float64, 'imu_correct', 10)
        self.imu_info = 0.0
        self.imu_corr = 0.0

    def listener_imu(self, msg):
        self.imu_info = float(msg.angular_velocity.z)

    def imu_correction(self):
        self.imu_corr += self.imu_info

    def timer_callback(self):
        tuple_of_topic = self.get_topic_names_and_types()
        topic_names = [i[0] for i in tuple_of_topic]
        msg = Float64()
        print(tuple_of_topic)
        if self.pico1_topic not in topic_names:
            print("lost signal with pico 1")
            self.reboot_callback(num=1)
            self.imu_correction()
            msg.data = self.imu_corr
            self.imu_correct.publish(msg)
        if self.pico2_topic not in topic_names:
            print("lost signal with pico 2")
            self.reboot_callback(num=2)

    def reboot_callback(self, num):
        if num == 1:
            GPIO.output(GPIO_PICO_1, GPIO.LOW)
            sleep(0.1)
            GPIO.output(GPIO_PICO_1, GPIO.HIGH)
        if num == 2:
            GPIO.output(GPIO_PICO_2, GPIO.LOW)
            sleep(0.1)
            GPIO.output(GPIO_PICO_2, GPIO.HIGH)


def main(args=None):
    rclpy.init(args=args)

    reboot_node = Reboot()

    rclpy.spin(reboot_node)

    reboot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
