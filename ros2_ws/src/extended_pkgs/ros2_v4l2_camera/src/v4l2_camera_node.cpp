#include "v4l2_camera/v4l2_camera.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<v4l2_camera::V4L2Camera>(rclcpp::NodeOptions{});

  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;

  return 0;
}
