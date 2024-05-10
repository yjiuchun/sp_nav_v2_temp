#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H
#include "executor/chassis.hpp"
#include "executor/gimbal.hpp"
#include "robot_msg/DecisionMsg.h"
#include "perception/blackboard.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include "tools/yaml_read.hpp"
namespace sp_decision
{
    class ControlNode
    {
    public:
        typedef std::shared_ptr<ControlNode> Ptr;
        std::mutex decision_cbk_mutex;
        int decision_status_;  // 决策动作状态
        ControlNode(const Blackboard::Ptr &blackboard_ptr, const tools::logger::Ptr &logger_ptr);
        ~ControlNode();
        void decision_sub(const robot_msg::DecisionMsg &decision);
        void execute_decision(); // 执行决策
        void points_init();      // 读取点集列表
        void run();              // 动作状态跟踪及更新
        void run_start();        // 启动决策执行器
        /**
         * @brief 决策一：冲锋
         *
         */
        void charge();
        /**
         * @brief 决策二：追踪。保持二到三米的距离，追踪超过5米原地停留
         *
         * @param enemy_id 追踪的敌方编号
         */
        Eigen::Vector2d init_pos;         // 开始追踪时的位置
        void enemy_filiter(int enemy_id); // 过滤追踪目标周围的点云
        void pursuit(int enemy_id);
        /**
         * @brief 决策三：返回补给点回血
         * 在决策树里设置状态位控制退出和进入
         */
        void add_blood();
        /**
         * @brief 决策四：返回启动区
         * decision_status_==3为到达
         */
        void reach_start_region();
       /**
         * @brief 加载points.yaml的含义
         * decision_status_==3为到达
         */
        enum Point_Yaml
        {
            ADD_area,
            ADD_AREA_ALTERNATE,
            ATTACK,
            RANGE_AREA1,
            RANGE_AREA2,
            RANGE_AREA3,
            RANGE_AREA4,
        };

    private:
        std::vector<Eigen::Vector2d> points_; // 读取点集列表
        bool control_thread_running;
        std::thread control_thread_;
        std::string decision_;           // 决策类别
        std::vector<double> param_list_; // 决策参数表
        int decision_type_;              // 为1时需要重置状态机
        std::string last_decision_;      // 决策类别
        ros::NodeHandle nh_;
        ros::Subscriber decision_sub_;                  // 决策信息订阅
        ros::Publisher enemy_pos_pub_;                  // 向点云处理程序发布
        // ros::Publisher robot_state_pub_;                // yjc 
        int loop_rate;                                  // 状态维护频率
        sp_decision::ChassisExecutor::Ptr chassis_ptr_; // 底盘控制
        sp_decision::GimbalExecutor::Ptr gimbal_ptr_;   // 云台控制
        sp_decision::Blackboard::Ptr blackboard_ptr_;
        tools::yaml_reader::Ptr yaml_reader_ptr_;
        tools::logger::Ptr logger_ptr_;
    };
}
#endif