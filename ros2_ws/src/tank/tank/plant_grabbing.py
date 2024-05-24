import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from time import sleep



class PlantGrabbing(Node):

    def __init__(self):
        super().__init__('plant_grabbing')
        self.publisher_stepper = self.create_publisher(Vector3, 'stepper_topic', 10)
        self.publisher_servo = self.create_publisher(Vector3, 'servo_topic', 10)
        self.screen_publisher = self.create_publisher(Vector3, 'screen_info', 10)
        self.publisher_response = self.create_publisher(String, 'main_response_topic', 10)
        self.command_listener = self.create_subscription(String, 'main_command_topic', self.act_decision, 10)
        self.subscriptions
        self.count = 0.0


    def act_decision(self, msg):
        if msg.data == "grab_the_plant":

            down = Vector3()
            down.x = 2.0
            down.y = 1.0
            down.z = 700.
            self.publisher_stepper.publish(down)
            sleep(4)
            
            grab = Vector3()
            grab.x = self.count + 1
            self.count += 1
            grab.y = 120.
            self.publisher_servo.publish(grab)
            screen = Vector3()
            screen.x = self.count 
            self.screen_publisher.publish(screen)
            sleep(4)

            up = Vector3()
            up.x = 2.0
            up.y = 2.0
            up.z = 700.
            self.publisher_stepper.publish(up)
            rot_drum = Vector3()
            rot_drum.x = 1.0
            rot_drum.y = 1.0
            rot_drum.z = 400.
            self.publisher_stepper.publish(rot_drum)
            sleep(3)
            resp = String()
            resp.data = "grab_done"
            self.publisher_response.publish(resp)
            

def main(args=None):
    rclpy.init(args=args)

    plant_grab = PlantGrabbing()
    rclpy.spin(plant_grab)

    plant_grab.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
