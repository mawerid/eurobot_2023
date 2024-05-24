import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import time


class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.command_listener = self.create_subscription(String, 'main_command_topic', self.act_decision, 10)
        self.scanner_response = self.create_publisher(String, 'main_response_topic', 10)
        self.publisher_aim = self.create_publisher(String, 'scan_topic', 10)
        self.subscriber_imu = self.create_subscription(Imu, 'imu_topic', self.listener_callback, 10)
        self.subscriber_plant = self.create_subscription(Vector3, 'plant_id', self.plant_callback, 10)
        self.nearest_plant = self.create_publisher(Vector3, 'nearest_plant', 10)
        self.subscriptions
        self.plant_dict = {}
        self.imu_info = 0
        self.start_rotate = 0
        self.plant_count = 0


    def listener_callback(self, msg):
        self.imu_info = float(msg.angular_velocity.z)

    def plant_callback(self, msg):
        self.plant_dict.update({ msg.x : [msg.y, self.imu_info]})
        print(self.plant_dict)

    def sort_plant(self):
        return dict(sorted(self.plant_dict.items(), key=lambda item: item[1][0]))

    def act_decision(self, msg):
        resp = String()
        rotate = String()
        if msg.data == "scan_space":
            rotate.data = 'rotate'
            self.publisher_aim.publish(rotate)
        elif msg.data == 'stop_scan':
            resp.data = "scanning_done"
            self.scanner_response.publish(resp)
            rotate.data = 'stop'
            self.publisher_aim.publish(rotate)
        elif msg.data == "give_nearest_plant":
            if self.count == 0:
                self.sorted_plant = self.sort_plant()
                ans = Vector3()
                key = list(self.sorted_plant.keys())[self.count]
                ans.x = float(key)
                ans.y = self.sorted_plant[key][0]
                ans.y = self.sorted_plant[key][1]
                self.count += 1
                self.nearest_plant.publish(ans)
            else:
                ans = Vector3()
                key = list(self.sorted_plant.keys())[self.count]
                ans.x = float(key)
                ans.y = self.sorted_plant[key][0]
                ans.y = self.sorted_plant[key][1]
                self.count += 1
                self.nearest_plant.publish(ans)
        else:
            print("I am waiting ...")




            
def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()
    rclpy.spin(scanner)

    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
