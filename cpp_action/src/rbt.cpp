#include "cpp_action/rbt.h"

float RoBot::move_step()
{
    int direct = move_distance_ / fabs(move_distance_);
    float step = direct * fabs(target_pose_ - current_pose_) * 0.1; /* 每一步移动当前到目标距离的1/10*/
    current_pose_ += step;
    std::cout << "移动了：" << step << "当前位置：" << current_pose_ << std::endl;
    return current_pose_;
}

bool RoBot::set_goal(float distance)
{
    move_distance_ = distance;
    target_pose_ += move_distance_;

    /* 当目标距离和当前距离大于0.01同意向目标移动 */
    if (close_goal()) 
    {
        status_ = MoveRobot::Feedback::STATUS_STOP;
        return false;
    }

    status_ = MoveRobot::Feedback::STATUS_MOVEING;
    return true;
}

float RoBot::get_current_pose()
{
    return current_pose_;
}

int RoBot::get_status()
{
    return status_;
}

bool RoBot::close_goal()
{
    return fabs(target_pose_ - current_pose_) < 0.01;
}

void RoBot::stop_move()
{
     status_ = MoveRobot::Feedback::STATUS_STOP;
}
