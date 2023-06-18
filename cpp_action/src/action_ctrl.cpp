#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "cpp_interfaces/action/move_action.hpp"

using namespace std::placeholders;

class ActionControl : public rclcpp::Node 
{
 public:
   using MoveRobot = cpp_interfaces::action::MoveAction;
   using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

    ActionControl(const std::string &name): Node(name) 
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
        action_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_rbt");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ActionControl::send_goal, this));
        cancel_timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                std::bind(&ActionControl::send_cancel, this));
    }
    void send_cancel()
    {
        cancel_timer_->cancel();
        if(sta == MoveRobot::Feedback::STATUS_MOVEING)
        {
            RCLCPP_INFO(this->get_logger(), "发送取消");
            try
            {
              action_client_->async_cancel_goal(g_handle,std::bind(&ActionControl::cancel_callback,this,_1));
            }
            catch(const std::exception &e)
            {
                std::cout<<e.what()<<std::endl;
            }
        }
    }
  void send_goal() 
  {

      this->timer_->cancel();

      if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) 
      {
          RCLCPP_ERROR(this->get_logger(),
                        "Action server not available after waiting");
          rclcpp::shutdown();
          return;
      }

      auto goal_msg = MoveRobot::Goal();
      goal_msg.distance = 10;

      RCLCPP_INFO(this->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();

      send_goal_options.goal_response_callback =
        std::bind(&ActionControl::goal_response_callback, this, _1);

      send_goal_options.feedback_callback =
        std::bind(&ActionControl::feedback_callback, this, _1, _2);

      send_goal_options.result_callback =
        std::bind(&ActionControl::result_callback, this, _1);

      this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }
private:
    void goal_response_callback(GoalHandleMoveRobot::SharedPtr goal_handle) 
    {
      if (!goal_handle) 
      {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else 
      {
          RCLCPP_INFO(this->get_logger(),"Goal accepted by server, waiting for result");
          g_handle = goal_handle;
      }
    }
    void feedback_callback(GoalHandleMoveRobot::SharedPtr ,
          const std::shared_ptr<const MoveRobot::Feedback> feedback) 
    {
      RCLCPP_INFO(this->get_logger(), "Feedback current pose:%f sta:%d", feedback->pose,feedback->status);
      sta = feedback->status;
    }
    void cancel_callback(rclcpp_action::Client<MoveRobot>::CancelResponse::SharedPtr cancel)
    {
        RCLCPP_INFO(this->get_logger(), "发送取消 的回调");
        {
            RCLCPP_INFO(this->get_logger(), "发送取消 的回调 取消成功 return_code:%d", cancel->return_code);
        }
    }
    void result_callback(const GoalHandleMoveRobot::WrappedResult& result) 
    {
        switch (result.code) 
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->pose);
         rclcpp::shutdown();
      }
private:
    rclcpp_action::Client<MoveRobot>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
    int sta = MoveRobot::Feedback::STATUS_STOP;
    GoalHandleMoveRobot::SharedPtr g_handle;
};  // class ActionControl01

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ActionControl>("node_action_ctrl");
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}
