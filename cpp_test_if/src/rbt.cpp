#include "rclcpp/rclcpp.hpp"
#include "cpp_interfaces/msg/rbt_sta.hpp"
#include "cpp_interfaces/srv/rbt_move.hpp"

using RbtSta_t = cpp_interfaces::msg::RbtSta;
using RbtMove_t = cpp_interfaces::srv::RbtMove;

/*创建一个机器人类，模拟真实机器人*/
class Rbt 
{
public:
  Rbt() = default;
  ~Rbt() = default;
  float move_distance(float distance) 
  {
    status_ = RbtSta_t::STATUS_MOVEING;
    target_pose_ += distance;
    // 当目标距离和当前距离大于0.01则持续向目标移动
    while (fabs(target_pose_ - current_pose_) > 0.01) 
    {
      // 每一步移动当前到目标距离的1/10
      float step = distance / fabs(distance) * fabs(target_pose_ - current_pose_) * 0.1;
      current_pose_ += step;
      std::cout << "移动了：" << step << "当前位置：" << current_pose_ << std::endl;
      // 当前线程休眠500ms
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    status_ = RbtSta_t::STATUS_STOP;
    return current_pose_;
  }
  float get_current_pose() { return current_pose_; }

  int get_status() { return status_; }

private:
     // 声明当前位置
  float current_pose_ = 0.0;
  // 目标距离
  float target_pose_ = 0.0;
  int status_ = RbtSta_t::STATUS_STOP;
};


class RbtNode : public rclcpp::Node {
public:
  RbtNode(std::string name) : Node(name) 
  {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
        /*创建move_robot服务*/
    move_robot_server_ = this->create_service<RbtMove_t>(
      "move_robot", std::bind(&RbtNode::handle_move_robot, this, std::placeholders::_1, std::placeholders::_2));
    /*创建发布者*/
    robot_status_publisher_ = this->create_publisher<RbtSta_t>("robot_status", 10);
    /*创建一个周期为500ms的定时器*/
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RbtNode::timer_callback, this));

  }
private:
    void timer_callback() 
    {
        // 创建消息
        RbtSta_t message;
        message.status = robot.get_status();
        message.pose = robot.get_current_pose();
        RCLCPP_INFO(this->get_logger(), "Publishing: %f", robot.get_current_pose());
        // 发布消息
        robot_status_publisher_->publish(message);
    }
    void handle_move_robot(const std::shared_ptr<RbtMove_t::Request> request,
                         std::shared_ptr<RbtMove_t::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(), "收到请求移动距离：%f,当前位置:%f", request->distance, robot.get_current_pose());
        robot.move_distance(request->distance);
        response->pose = robot.get_current_pose();
    }
private:
  Rbt robot;
  rclcpp::TimerBase::SharedPtr timer_; /*定时器，用于定时发布机器人位置*/
  rclcpp::Service<RbtMove_t>::SharedPtr move_robot_server_; /*移动机器人服务*/
  rclcpp::Publisher<RbtSta_t>::SharedPtr robot_status_publisher_; /*发布机器人位姿发布者*/

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbtNode>("rbt_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
