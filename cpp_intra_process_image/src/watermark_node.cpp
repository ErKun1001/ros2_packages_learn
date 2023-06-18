#include <memory>

#include "cpp_intra_process_image/watermark_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto watermark_node =
    std::make_shared<WatermarkNode>("image", "watermarked_image", "Hello world!");
  rclcpp::spin(watermark_node);

  rclcpp::shutdown();

  return 0;
}