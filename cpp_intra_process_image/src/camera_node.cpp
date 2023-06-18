#include <memory>

#include "cpp_intra_process_image/camera_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<CameraNode> camera_node = nullptr;
  try {
    camera_node = std::make_shared<CameraNode>("image");
  } catch (const std::exception & e) {
    fprintf(stderr, "%s Exiting..\n", e.what());
    return 1;
  }

  rclcpp::spin(camera_node);

  rclcpp::shutdown();

  return 0;
}