import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from camera_info_msgs.msg import CameraInfo
from nav_msgs.msg import OccupancyGrid  # Import OccupancyGrid message


class OccupancyGridGenerator(Node):

    def __init__(self):
        super().__init__('occupancy_grid_generator')

        # ROS2 parameter configuration (consider adding these to your launch file)
        self.image_topic = self.get_parameter('image_topic', default='/camera/image_raw')
        self.camera_info_topic = self.get_parameter('camera_info_topic', default='/camera/camera_info')
        self.gridmap_topic = self.get_parameter('gridmap_topic', default='/occupancy_grid')
        self.cell_size = self.get_parameter('cell_size', default=0.05)  # Meters per cell

        # Image subscriber and publisher initialization
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)

        # Camera info subscriber (if available)
        self.camera_info_subscription = None
        if self.has_parameter('camera_info_topic'):
            self.camera_info_subscription = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                10)

        # Occupancy grid publisher
        self.gridmap_publisher = self.create_publisher(OccupancyGrid, self.gridmap_topic, 10)

        # Internal variables
        self.image = None
        self.camera_info = None
        self.gridmap = None

    def image_callback(self, msg):
        self.image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def camera_info_callback(self, msg):
        self.camera_info = msg  # Store camera information for potential use

    def create_gridmap(self):
        if self.image is None:
            self.get_logger().warn("No image received yet. Waiting...")
            return

        # Process image to segment foreground (working area) and background (borders)
        # Here, you can implement your preferred segmentation algorithm (e.g., color thresholding, background subtraction)
        # Assuming a simple red-based segmentation for illustration:
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([170, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Extract image dimensions (assuming rectified image)
        height, width, _ = self.image.shape

        # Define gridmap parameters based on image size and cell size
        gridmap_width = int(width * self.cell_size)
        gridmap_height = int(height * self.cell_size)
        origin_x = -gridmap_width * self.cell_size / 2.0
        origin_y = -gridmap_height * self.cell_size / 2.0

        # Create occupancy grid data structure
        self.gridmap = OccupancyGrid()
        self.gridmap.header.frame_id = 'camera_link'  # Adjust frame_id if needed
        self.gridmap.info.resolution = self.cell_size
        self.gridmap.info.width = gridmap_width
        self.gridmap.info.height = gridmap_height
        self.gridmap.info.origin.position.x = origin_x
        self.gridmap.info.origin.position.y = origin_y

        # Populate gridmap data with occupancy values based on segmentation mask
        data = []
        for y in range(gridmap_height):
            for x in range(gridmap_width):
                # Convert image coordinates to gridmap indices (adjusted for potential offset)
                image_x = int(x / self.cell_size)
                image_y = int(y / self.cell_size)

                # Check segmentation mask value at corresponding image pixel
                if mask[image_y, image_x] > 0:  # Working area (likely red)
                    data.append(-1)  # Occupied cell (-1 for unknown or occupied)
                else:
                    data.append(0)  # Free cell (0 for free)

        self.gridmap.data = data

        # Publish the occupancy grid
        self.gridmap_publisher.publish(self.gridmap)

    def main(args=None):
        rclpy.init(args=args)
        node = OccupancyGridGenerator()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
