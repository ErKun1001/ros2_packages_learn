#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class Cli :public rclcpp::Node
{
    public:
        using srv_t = example_interfaces::srv::AddTwoInts;
        Cli(const std::string &name):Node(name)
        {
            RCLCPP_INFO(this->get_logger(),"节点%s启动!\n",name.c_str());
            _cli = this->create_client<srv_t>("my_add_int");
        }
        void send_request(int a, int b)
        {
            RCLCPP_INFO(this->get_logger(),"request %d + %d\n",a,b);
            while(!_cli->wait_for_service(std::chrono::seconds(1)))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(this->get_logger(),"wait srv but break\n");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),"wait srv online...\n");
            }
            auto request = std::make_shared<srv_t::Request>();
            request->a = a;
            request->b = b;
            _cli->async_send_request(request,std::bind(&Cli::result_callback_, this,std::placeholders::_1));
        }
    private:
        rclcpp::Client<srv_t>::SharedPtr _cli;
        void result_callback_(rclcpp::Client<srv_t>::SharedFuture result_future) 
        {
                auto response = result_future.get();
                RCLCPP_INFO(this->get_logger(), "计算结果：%ld", response->sum);
        }
};

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto cli = std::make_shared<Cli>("my_cli");
    cli->send_request(1,3);
    rclcpp::spin(cli);
    rclcpp::shutdown();
    return 0;
}
