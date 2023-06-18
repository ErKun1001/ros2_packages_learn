#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class Srv :public rclcpp::Node
{
    public:
        using srv_t = example_interfaces::srv::AddTwoInts;
        Srv(const std::string &name):Node(name)
        {
            RCLCPP_INFO(this->get_logger(),"节点%s启动!\n",name.c_str());
            _srv = this->create_service<srv_t>("my_add_int",
                    std::bind(&Srv::handle_add_two_ints,this,std::placeholders::_1,std::placeholders::_2));
        }
    private:
        rclcpp::Service<srv_t>::SharedPtr _srv;

        void handle_add_two_ints(
            const std::shared_ptr<srv_t::Request> request,
            std::shared_ptr<srv_t::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a,
                    request->b);
        response->sum = request->a + request->b;
    };
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Srv>("my_srv"));
    rclcpp::shutdown();
    return 0;
}
