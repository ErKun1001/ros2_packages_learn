cmake_minimum_required(VERSION 3.8)
project(cpp_intra_process_image)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

include_directories(include)

# A single program with one of each of the image pipeline demo nodes.
add_executable(all_in_one
  src/all_in_one.cpp)
target_link_libraries(all_in_one
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui)

# A single program with one of each of the image pipeline demo nodes, but two image views.
add_executable(two_image_view
  src/two_image_view.cpp)
target_link_libraries(two_image_view
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui)

# A stand alone node which produces images from a camera using OpenCV.
add_executable(camera_node
  src/camera_node.cpp)
target_link_libraries(camera_node
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui
  opencv_imgproc)

# A stand alone node which adds some text to an image using OpenCV before passing it along.
add_executable(watermark_node
  src/watermark_node.cpp)
target_link_libraries(watermark_node
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_videoio)

# A stand alone node which consumes images and displays them using OpenCV.
add_executable(image_view_node
  src/image_view_node.cpp)
target_link_libraries(image_view_node
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui)

  install(TARGETS
  all_in_one
  two_image_view
  camera_node
  watermark_node
  image_view_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
