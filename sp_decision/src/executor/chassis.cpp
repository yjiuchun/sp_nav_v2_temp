#include "executor/chassis.hpp"
namespace sp_decision
{
    ChassisExecutor::ChassisExecutor(const tools::logger::Ptr &logger_ptr, const sp_decision::Blackboard::Ptr &blackboard_ptr)
    {
        logger_ptr_ = logger_ptr; // 获取日志器
        blackboard_ptr_ = blackboard_ptr;
        set_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        sentry_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/sentry/cmd_vel", 1);
        robot_state_pub_ = nh_.subscribe("robot_state", 10, &ChassisExecutor::robot_state_sub, this);
        rotate_state_pub_ = nh_.subscribe("rotate_state", 10, &ChassisExecutor::rotate_state_sub, this);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &ChassisExecutor::cmd_vel_callback, this);
        localization_sub_=nh_.subscribe("localization", 10, &ChassisExecutor::localization_callback, this);
        enemy_pos_pub_ = nh_.advertise<geometry_msgs::Pose>("/need_filterate/enemy_pos", 10);
        rotate_state_ = RotateState::ROTATE;
        target_pose_.pose.position.x = 100;
        target_pose_.pose.position.y = 100; // 确保初值不会和第一个点冲突--------待优化
    }
    void ChassisExecutor::rotate_state_sub(const robot_msg::RobotStateMsg &rotate_state)
    {
        rotate_state_cbk_mutex.lock();
        rotate_state_ = static_cast<RotateState>(rotate_state.robot_state);
        rotate_state_cbk_mutex.unlock();
    }
    void ChassisExecutor::robot_state_sub(const robot_msg::RobotStateMsg &robot_state)
    {
        robot_state_cbk_mutex.lock();
        robot_state_ = static_cast<RobotState>(robot_state.robot_state);
        robot_state_cbk_mutex.unlock();
    }
    void ChassisExecutor::cmd_vel_callback(const geometry_msgs::Twist &msg)
    {
        nav_cmd_vel_cbk_mutex.lock();
        cmd_vel_.linear.x = msg.linear.x;
        cmd_vel_.linear.y = msg.linear.y;
        cmd_vel_.angular.z = msg.angular.z;
        geometry_msgs::Twist sentry_cmd_vel;

        switch (robot_state_)
        {
        case RobotState::SLOW:{
            sentry_cmd_vel.linear.x = std::min(cmd_vel_.linear.x, 0.4);
            sentry_cmd_vel.linear.y = std::min(cmd_vel_.linear.y, 0.4);
            sentry_cmd_vel.angular.z = 2 * static_cast<int>(rotate_state_);
            break;
        }
        case RobotState::MOVE:{
            sentry_cmd_vel.linear.x = std::min(cmd_vel_.linear.x, 1.0);
            sentry_cmd_vel.linear.y = std::min(cmd_vel_.linear.y, 1.0);
            sentry_cmd_vel.angular.z = 2 * static_cast<int>(rotate_state_);
            break;
        }
        case RobotState::FAST:{
            sentry_cmd_vel.linear.x = std::min(cmd_vel_.linear.x * 2, 3.0);
            sentry_cmd_vel.linear.y = std::min(cmd_vel_.linear.y * 2, 3.0);
            sentry_cmd_vel.angular.z = 2 * static_cast<int>(rotate_state_);
            break;
        }
        case RobotState::STOP:{
            sentry_cmd_vel.linear.x = 0;
            sentry_cmd_vel.linear.y = 0;
            sentry_cmd_vel.angular.z = 2 * static_cast<int>(rotate_state_);
            break;
        }
        default:{
            break;
        }
        }
        sentry_cmd_vel_pub_.publish(sentry_cmd_vel);
        nav_cmd_vel_cbk_mutex.unlock();
    }
    void ChassisExecutor::localization_callback(const nav_msgs::Odometry::ConstPtr msg)
    {
        localization_cbk_mutex.lock();
        localization_ = *msg;
        localization_cbk_mutex.unlock();
    }
    int ChassisExecutor::send_goal(double pos_x, double pos_y)
    {
        double d=sqrt(pow(pos_x - localization_.pose.pose.position.x, 2) + pow(pos_y - localization_.pose.pose.position.y, 2));
        std::cout<<"d="<<d<<std::endl;
        // 距离小于0.2m认为到达
        if (d< 0.18)
        {
            return 2;
        }
        else
        {
            if (target_pose_.pose.position.x == pos_x && target_pose_.pose.position.y == pos_y)
            {
                ros::Duration time_interval = ros::Time::now() - last_judge_time_;
                double time_interval_ms = time_interval.toSec() * 1000.0; // 计算时间间隔
                if (time_interval_ms > 100)                               // 每100ms进行一次判断
                {
                    if ((cmd_vel_.linear.x == last_cmd_vel_.linear.x && cmd_vel_.linear.y == last_cmd_vel_.linear.y) ||
                        (cmd_vel_.linear.x == 0 && cmd_vel_.linear.y == 0))
                    {
                        last_judge_time_ = ros::Time::now();
                        last_cmd_vel_.linear.x = cmd_vel_.linear.x;
                        last_cmd_vel_.linear.y = cmd_vel_.linear.y;
                        directly_send_goal(pos_x, pos_y);
                        return 0;
                    }
                    else
                    {
                        last_judge_time_ = ros::Time::now();
                        last_cmd_vel_.linear.x = cmd_vel_.linear.x;
                        last_cmd_vel_.linear.y = cmd_vel_.linear.y;
                        directly_send_goal(pos_x, pos_y);
                        return 1;
                    }
                }
                else
                {
                    if (nav_status_ == 0)
                    {
                        nav_status_ = 0;
                        return 0;
                    }
                    else
                    {
                        return 1;
                    }
                }
            }
            else
            {
                last_judge_time_ = ros::Time::now();
                last_cmd_vel_.linear.x = cmd_vel_.linear.x;
                last_cmd_vel_.linear.y = cmd_vel_.linear.y;
                directly_send_goal(pos_x, pos_y);
                nav_status_ = 1;
                return 1;
            }
        }
    }
    void ChassisExecutor::directly_send_goal(double pos_x, double pos_y)
    {
        target_pose_.header.frame_id = "map";
        target_pose_.header.stamp = ros::Time::now();
        target_pose_.pose.position.x = pos_x;
        target_pose_.pose.position.y = pos_y;
        target_pose_.pose.orientation.x = 0.0;
        target_pose_.pose.orientation.y = 0.0;
        target_pose_.pose.orientation.z = 0.0;
        target_pose_.pose.orientation.w = 1;
        goal_.target_pose = target_pose_;
        set_goal_pub_.publish(goal_);
    }
    // 基本动作1——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
    int ChassisExecutor::range_move(std::vector<Eigen::Vector2d> points)
    {
        Eigen::Vector2d target;
        int result;
        static int wait_time_flag = 0;
        switch (action_status_)
        {
        case 0: // 初始化
            target = point_generate(points);
            wait_time_flag = 0;
            if (send_goal(target[0], target[1]) == 1)
            {
                action_status_ = 2;
                last_target = target;
            }
            else
            {
                rotate_inplace();
                action_status_ = 1;
            }
            break;
        case 1: // 搜寻可行域
            target = point_generate(points);
            if (send_goal(target[0], target[1]) == 1)
            {
                action_status_ = 2;
                last_target = target;
            }
            else
            {
                rotate_inplace();
                action_status_ = 1;
            }
            break;
        case 2: // 前往
            result = send_goal(last_target[0], last_target[1]);
            if (result == 0)
            {
                rotate_inplace();
                action_status_ = 1;
            }
            else if (result == 1)
                action_status_ = 2;
            else if (result == 2)
                action_status_ = 3;
            break;
        case 3: // 到达
        {
            send_goal(last_target[0], last_target[1]);
            if(!wait_time_flag){
                range_wait_time_ = ros::Time::now();
            }
            ros::Duration time_interval_range = ros::Time::now() - range_wait_time_;
            double time_interval_ms_range = time_interval_range.toSec() * 1000.0; // 计算时间间隔
            wait_time_flag = 1;
            if(time_interval_ms_range > 3000){
            action_status_ = 0;
            }
            break;
        }
        case 4: // 失败
            break;
        default:
            break;
        }
    }
    bool isInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point)
    {
        bool inside = false;
        int i, j = polygon.size() - 1;
        for (i = 0; i < polygon.size(); i++)
        {
            if ((polygon[i][1] < point[1] && polygon[j][1] >= point[1] || polygon[j][1] < point[1] && polygon[i][1] >= point[1]) &&
                (polygon[i][0] <= point[0] || polygon[j][0] <= point[0]))
            {
                inside ^= (polygon[i][0] + (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) * (polygon[j][0] - polygon[i][0]) < point[0]);
            }
            j = i;
        }
        return inside;
    }
    Eigen::Vector2d ChassisExecutor::point_generate(std::vector<Eigen::Vector2d> points)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::min();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::min();
        for (const Eigen::Vector2d &p : points)
        {
            if (p[0] < minX)
                minX = p[0];
            if (p[0] > maxX)
                maxX = p[0];
            if (p[1] < minY)
                minY = p[1];
            if (p[1] > maxY)
                maxY = p[1];
        }
        // 定义生成随机数的分布
        std::uniform_real_distribution<> disX(minX, maxX);
        std::uniform_real_distribution<> disY(minY, maxY);
        // 随机生成点，直到生成的点在多边形内部
        Eigen::Vector2d randomPoint;
        do
        {
            randomPoint[0] = disX(gen);
            randomPoint[1] = disY(gen);
        } while (!isInsidePolygon(points, randomPoint));
        return randomPoint;
    }
    // 基本动作2——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
    void ChassisExecutor::rotate_inplace()
    {
        robot_state_ = RobotState::SLOW;
        rotate_state_ = RotateState::ROTATE;
        send_goal(localization_.pose.pose.position.x, localization_.pose.pose.position.y);
    }
    // 基本动作3——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
    void ChassisExecutor::cequence_move(std::vector<Eigen::Vector2d> points, std::vector<float> wait_time, bool circulate)
    {
        int n = action_status_ / 3;
        switch (action_status_ % 3)
        {
        case 0:
            if (circulate == true && n == points.size()) // 进入循环
            {
                action_status_ = 0;
                n = 0;
            }
            if (wait_time[n] > 0 && n > 0) // 到达后停留
            {
                rotate_inplace();
                wait_time[n - 1] -= (ros::Time::now() - last_reach_time).sec;
            }
            if (send_goal(points[n][0], points[n][1]) == 1)
                action_status_ += 1;
            else
            {
                rotate_inplace();
                action_status_ += 2;
            }
            break;
        case 1:
            if (send_goal(points[n][0], points[n][1]) == 0)
            {
                rotate_inplace();
                action_status_ += 1;
            }
            else if (send_goal(points[n][0], points[n][1]) == 2)
            {
                rotate_inplace();
                action_status_ += 2;
                last_reach_time = ros::Time::now();
            }
            break;
        case 2:
            if (send_goal(points[n][0], points[n][1]) == 1)
            {
                action_status_ -= 1;
            }
            break;
        default:
            break;
        }
    }
    // 基本动作4——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
    void ChassisExecutor::single_point_move(Eigen::Vector2d point, Eigen::Vector2d alternate_point,double* onarrive_flag)
    {
        ROS_INFO("status------%d",action_status_);
        switch (action_status_)
        {
        case 0:
            if (send_goal(point[0], point[1]) == 1)
            {
                action_status_ = 1;
            }
            else if (send_goal(alternate_point[0], alternate_point[1]) == 1)
            {
                action_status_ = 3;
            }
            else
            {
                rotate_inplace();
                action_status_ = 5;
            }
            break;
        case 1:
            if (send_goal(point[0], point[1]) == 2)
            {
                rotate_inplace();
                action_status_ = 2;
                *onarrive_flag = 1;

            }
            else if (send_goal(point[0], point[1]) == 0)
            {
                if (send_goal(alternate_point[0], alternate_point[1]) == 1)
                {
                    action_status_ = 3;
                }
                else
                {
                    rotate_inplace();
                    action_status_ = 5;
                }
            }
            break;
        case 2:
            send_goal(point[0], point[1]);
            // rotate_inplace();
            action_status_ = 2;
            break;
        case 3:
            if (send_goal(alternate_point[0], alternate_point[1]) == 2)
            {
                action_status_ = 4;
            }
            else if (send_goal(alternate_point[0], alternate_point[1]) == 0)
            {
                rotate_inplace();
                action_status_ = 5;
            }
            break;
        case 4:
            if (send_goal(point[0], point[1]) == 1)
            {
                action_status_ = 1;
            }
            else
            {
                rotate_inplace();
                action_status_ = 5;
            }
        case 5:
            if (send_goal(point[0], point[1]) == 1)
            {
                action_status_ = 1;
            }
            else if (send_goal(alternate_point[0], alternate_point[1]) == 1)
            {
                action_status_ = 3;
            }
            else
            {
                rotate_inplace();
                action_status_ = 5;
            }
            break;
        default:
            break;
        }
    }
    void ChassisExecutor::pursuit(int enemy_id)
    {
        geometry_msgs::Pose enemy_poss;
        enemy_poss.position.x = blackboard_ptr_->enemy_status[enemy_id].robot_pos[0];
        enemy_poss.position.y = blackboard_ptr_->enemy_status[enemy_id].robot_pos[1];
        double d = sqrt(pow(blackboard_ptr_->sentry_status.robot_pos[0] - blackboard_ptr_->enemy_status[enemy_id].robot_pos[0], 2) +
                        pow(blackboard_ptr_->sentry_status.robot_pos[1] - blackboard_ptr_->enemy_status[enemy_id].robot_pos[1], 2));
        if ((ros::Time::now().sec - blackboard_ptr_->enemy_status[enemy_id].pos_update_time.sec) < 0.5) // 0.5秒即超时
        {
            enemy_pos_pub_.publish(enemy_poss);
            if (d < 1.5)
            {
                rotate_inplace();
            }
            else
                send_goal(enemy_poss.position.x, enemy_poss.position.y);
        }
    }

}