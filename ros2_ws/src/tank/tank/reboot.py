import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from time import sleep

# Сисема нумерации пинов на плате:
        #по номеру пина на гребенке коннектора GPIO, например: 26 (IO7), 40 (IO21)
GPIO.setmode(GPIO.BOARD)

class Reboot(Node):

    def __init__(self):
        super().__init__('reboot')
        self.i = 0
        self.pinnum_pico_motor = 40
        # Установка канала на работу "на выход"
        GPIO.setup(self.pinnum_pico_motor, GPIO.OUT, initial=GPIO.HIGH)
        self.reboot_process = self.reboot_callback()
        
        

    def reboot_callback(self):
            GPIO.output(self.pinnum_pico_motor, GPIO.LOW)
            sleep(1)
            GPIO.cleanup(self.pinnum_pico_motor)
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    reboot_node = Reboot()

    try:
        rclpy.spin(reboot_node)
    except SystemExit:
        print('Done')

    reboot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()