#include "rclcpp/rclcpp.hpp"

class Param : public rclcpp::Node
{
public:
    Param(const std::string &name):Node(name)
    {
        this->declare_parameter("log_level",0);
        RCLCPP_INFO(this->get_logger(),"Node (%s) start",name.c_str());
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&Param::timer_callback, this));
    }
private:
    void timer_callback()
    {
        this->get_parameter("log_level",log_level_);
        this->get_logger().set_level(static_cast<rclcpp::Logger::Level>(log_level_));
        
        RCLCPP_DEBUG(this->get_logger(),"DEBUG level print");
        RCLCPP_INFO(this->get_logger(),"INFO level print");
        RCLCPP_WARN(this->get_logger(),"WARN level print");
        RCLCPP_ERROR(this->get_logger(),"ERROR level print");
        RCLCPP_FATAL(this->get_logger(),"FATAL level print");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    int log_level_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Param>("node_my_param"));
    rclcpp::shutdown();
    return 0;
}