#include "cpp_action/rbt.h"

using namespace std::placeholders;


// 创建一个ActionServer类
class ActionRobot : public rclcpp::Node 
{
public:
    using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

    ActionRobot(const std::string &name) : Node(name) 
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

        action_server_ = rclcpp_action::create_server<MoveRobot>(this,"move_rbt",
                std::bind(&ActionRobot::handle_goal, this, _1, _2),
                std::bind(&ActionRobot::handle_cancel, this, _1),
                std::bind(&ActionRobot::handle_accepted, this, _1));
    }
private:
    rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const MoveRobot::Goal> goal) 
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with distance %f",
                    goal->distance);
        (void)uuid;
        if (fabs(goal->distance > 100)) 
        {
            RCLCPP_WARN(this->get_logger(), "目标距离太远了，本机器人表示拒绝！");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(),
                    "目标距离%f我可以走到，本机器人接受，准备出发！",
                    goal->distance);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleMoveRobot> goal_handle) 
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        robot.stop_move(); /*认可取消执行，让机器人停下来*/
        return rclcpp_action::CancelResponse::ACCEPT;
   }
    void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) 
    {
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "开始执行移动 %f 。。。", goal->distance);

        auto result = std::make_shared<MoveRobot::Result>();
        rclcpp::Rate rate = rclcpp::Rate(2);
        robot.set_goal(goal->distance);
        while (rclcpp::ok() && !robot.close_goal()) 
        {
            robot.move_step();
            auto feedback = std::make_shared<MoveRobot::Feedback>();
            feedback->pose = robot.get_current_pose();
            feedback->status = robot.get_status();
            goal_handle->publish_feedback(feedback);
            /*检测任务是否被取消*/
            if (goal_handle->is_canceling()) 
            {
                result->pose = robot.get_current_pose();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/
            rate.sleep();
        }

        result->pose = robot.get_current_pose();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        rclcpp::shutdown();
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) 
    {
        std::thread{std::bind(&ActionRobot::execute_move, this, _1), goal_handle}.detach();
    }

private:
    RoBot robot;
    rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
};

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<ActionRobot>("node_action_robot");
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}
