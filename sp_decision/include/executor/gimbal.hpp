/**
 * @file gimbal.hpp
 * @author lp
 * @brief 发布云台控制量
 * @version 0.1
 * @date 2024-04-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef GIMBAL_EXECUTOR_H
#define GIMBAL_EXECUTOR_H

#include <ros/ros.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_msg/CmdGimbal.h>
#include "tools/log.hpp"
namespace sp_decision
{
    class GimbalExecutor
    {
    public:
        typedef std::shared_ptr<GimbalExecutor> Ptr;
        std::mutex robot_state_cbk_mutex;
        GimbalExecutor(const tools::logger::Ptr &logger_ptr);
        void gimbal_set(double min_angle, double max_angle, bool pitch_enable);

    private:
        tools::logger::Ptr logger_ptr_;
        ros::NodeHandle nh_;
        ros::Publisher gimbal_pub_;
    };
}
#endif