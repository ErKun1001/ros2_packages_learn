#include <memory>

#include "cpp_intra_process_image/image_view_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto image_view_node = std::make_shared<ImageViewNode>("watermarked_image");
  rclcpp::spin(image_view_node);

  rclcpp::shutdown();

  return 0;
}