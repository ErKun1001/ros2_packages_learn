#ifndef __R_B_T_H__
#define __R_B_T_H__

#include "rclcpp/rclcpp.hpp"
#include "cpp_interfaces/action/move_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using MoveRobot = cpp_interfaces::action::MoveAction;

class RoBot
{
public:

    RoBot() = default;
    ~RoBot() = default;
    float move_step();
    bool set_goal(float goal);
    float get_current_pose();
    int get_status();
    bool close_goal();
    void stop_move();
private:
    float current_pose_ = 0.0;             /*声明当前位置*/
    float target_pose_ = 0.0;              /*目标距离*/
    float move_distance_ = 0.0;            /*目标距离*/
    std::atomic<bool> cancel_flag_{false}; /*取消标志*/
    int status_ = MoveRobot::Feedback::STATUS_STOP;
};

#endif