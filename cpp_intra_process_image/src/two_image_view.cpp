#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "cpp_intra_process_image/camera_node.hpp"
#include "cpp_intra_process_image/image_view_node.hpp"
#include "cpp_intra_process_image/watermark_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // Connect the nodes as a pipeline: camera_node -> watermark_node -> image_view_node
  // And the extra image view as a fork:                           \-> image_view_node2
  std::shared_ptr<CameraNode> camera_node = nullptr;
  try {
    camera_node = std::make_shared<CameraNode>("image");
  } catch (const std::exception & e) {
    fprintf(stderr, "%s Exiting..\n", e.what());
    return 1;
  }
  auto watermark_node =
    std::make_shared<WatermarkNode>("image", "watermarked_image", "Hello world!");
  auto image_view_node = std::make_shared<ImageViewNode>("watermarked_image");
  auto image_view_node2 = std::make_shared<ImageViewNode>("watermarked_image", "image_view_node2");

  executor.add_node(camera_node);
  executor.add_node(watermark_node);
  executor.add_node(image_view_node);
  executor.add_node(image_view_node2);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}