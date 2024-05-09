/**
 * @file chassis.hpp
 * @author lp
 * @brief 在此程序进行速度转发，因此需要接收robotstate
 * @version 0.1
 * @date 2024-04-06
 * @copyright Copyright (c) 2024
 */
#ifndef CHASSIS_EXECUTOR_H
#define CHASSIS_EXECUTOR_H

#include <ros/ros.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_msg/RobotStateMsg.h>
#include <Eigen/Geometry>
#include <random>

#include "robot_msg/CmdGimbal.h"
#include "tools/log.hpp"
#include "perception/blackboard.hpp"
namespace sp_decision
{

    // 给MoveBaseAction定义一个别名，方便创建对象
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    class ChassisExecutor
    {
    public:
        typedef std::shared_ptr<ChassisExecutor> Ptr;
        int action_status_; // 动作状态,每次进入新动作需初始化为0
        std::mutex robot_state_cbk_mutex;
        std::mutex rotate_state_cbk_mutex;
        std::mutex nav_cmd_vel_cbk_mutex;
        std::mutex localization_cbk_mutex;
        // 导航模式
        enum RobotState
        {
            SLOW,
            MOVE,
            FAST,
            STOP,
        };
        // 小陀螺模式
        enum RotateState
        {
            IDLE,
            UPSLOPW,
            ROTATE,
        };
        ChassisExecutor(const tools::logger::Ptr &logger_ptr, const sp_decision::Blackboard::Ptr &blackboard_ptr);
        int send_goal(double pos_x, double pos_y);           // 0为失败，1为进行中，2为成功到达
        void directly_send_goal(double pos_x, double pos_y); // 直接发布目标点
        void robot_state_sub(const robot_msg::RobotStateMsg &robot_state);
        void rotate_state_sub(const robot_msg::RobotStateMsg &rotate_state);
        void cmd_vel_callback(const geometry_msgs::Twist &msg);             // 接收move_base规划的速度
        void localization_callback(const nav_msgs::Odometry::ConstPtr msg); // 接受定位信息
        /**
         * @brief 基本动作1，范围导航
         *
         * @param points 至少三个点，在围成的图形中选取可行的目标点，每个位置等价值
         * action_status_定义：0为未初始化，1为搜寻可行点，2为前往目标区域，3为成功，4为长时间无法找到可行点
         * @todo 超时后关闭动态避障
         */
        int range_move(std::vector<Eigen::Vector2d> points);
        Eigen::Vector2d last_target;                                         // 记录上次目标点
        Eigen::Vector2d point_generate(std::vector<Eigen::Vector2d> points); // 随机点生成器
        /**
         * @brief 基本动作2，原地小陀螺
         */
        void rotate_inplace();
        RotateState rotate_state_; // 小陀螺模式
        RobotState robot_state_; // 导航模式

        /**
         * @brief 基本功能3，序列点导航：依次通过每个目标点，当被阻挡时原地停留
         *
         * @param points 依次导航目标点
         * @param wait_time 对应在每个点停留的时间,/s
         * @param circulate 是否循环
         */
        void cequence_move(std::vector<Eigen::Vector2d> points, std::vector<float> wait_time, bool circulate);
        ros::Time last_reach_time; // 到达一个点后开始计时
        /**
         * @brief 基本功能4，单点导航：当目标点不可用时前往备用点，否则原地自旋
         *
         * @param point 最终目标点
         * @param alternate_point 备用点
         */
        void single_point_move(Eigen::Vector2d point, Eigen::Vector2d alternate_point, double* onarrive_flag);
        /**
         * @brief 基本功能5，追踪
         *
         * @param enemy_id 敌方编号
         */
        void pursuit(int enemy_id);

    private:
        // RobotState robot_state_; // 导航模式

        geometry_msgs::Twist cmd_vel_;      // move_base规划的速度
        geometry_msgs::Twist last_cmd_vel_; // move_base规划的速度
        nav_msgs::Odometry localization_;   // 定位信息
        ros::Time last_judge_time_;         // 判断路径是否存在的计时器
        ros::Time range_wait_time_;         //judge when to a new goal
        int nav_status_;                    // 导航状态
        tools::logger::Ptr logger_ptr_;
        sp_decision::Blackboard::Ptr blackboard_ptr_;
        ros::NodeHandle nh_;
        ros::Publisher set_goal_pub_;       // 导航目标发布
        ros::Subscriber robot_state_pub_;   // 导航模式接收
        ros::Subscriber rotate_state_pub_;  // 小陀螺模式接收
        ros::Publisher sentry_cmd_vel_pub_; // 速度转发
        ros::Publisher enemy_pos_pub_;      // 向雷达发送过滤信息
        ros::Subscriber cmd_vel_sub_;       // move_base速度规划器
        ros::Subscriber localization_sub_;  // 接受定位信息
        geometry_msgs::PoseStamped target_pose_;
        move_base_msgs::MoveBaseGoal goal_;
    };
}
#endif