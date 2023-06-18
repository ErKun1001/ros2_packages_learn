#include "rclcpp/rclcpp.hpp"
#include "cpp_interfaces/msg/rbt_sta.hpp"
#include "cpp_interfaces/srv/rbt_move.hpp"

using RbtSta_t = cpp_interfaces::msg::RbtSta;
using RbtMove_t = cpp_interfaces::srv::RbtMove;

class RbtCtrl : public rclcpp::Node {
public:
    // 构造函数,有一个参数为节点名称
    RbtCtrl(std::string name) : Node(name) 
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
        client_ = this->create_client<RbtMove_t>("move_robot");
        robot_status_subscribe_ = this->create_subscription<RbtSta_t>("robot_status",10,
                        std::bind(&RbtCtrl::robot_status_callback_,this,std::placeholders::_1));
    }

    void move_robot(float distance)
    {
        RCLCPP_INFO(this->get_logger(),"request distance:%f\n",distance);
        while(!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(),"wait srv but break\n");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"wait srv online...\n");
        }
        auto request = std::make_shared<RbtMove_t::Request>();
        request->distance = distance;
        client_->async_send_request(request,std::bind(&RbtCtrl::result_callback_, this,std::placeholders::_1));
    }
private:
  /* 机器人移动结果回调函数 */
    void result_callback_(rclcpp::Client<RbtMove_t>::SharedFuture result_future) 
    {
        auto response = result_future.get();
        RCLCPP_INFO(this->get_logger(), "收到移动结果：%f", response->pose);
    }
    void robot_status_callback_(const RbtSta_t::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到状态数据位置：%f 状态：%d", msg->pose ,msg->status);
    }
private:
  // 声明客户端
  rclcpp::Client<RbtMove_t>::SharedPtr client_;
  rclcpp::Subscription<RbtSta_t>::SharedPtr robot_status_subscribe_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbtCtrl>("rbt_ctrl");
  node->move_robot(5.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
