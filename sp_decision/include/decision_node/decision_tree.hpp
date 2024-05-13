/**
 * @file decision_tree.hpp
 * @author lp
 * @brief 调用yaml_read读取决策文件，生成决策二叉树，为decision_node提供接口，把决策树的搜寻放到新线程运行
 * @version 0.1
 * @date 2024-04-07
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef DECISION_NODE_H
#define DECISION_NODE_H

#include <iostream>
#include <chrono> // For std::chrono
#include <thread> // For std::this_thread::sleep_for
#include <ros/ros.h>
#include <signal.h>
#include <ros/package.h>

#include "robot_msg/DecisionMsg.h"
#include "perception/blackboard.hpp"
#include "tools/yaml_read.hpp"

namespace sp_decision
{
    /**
     * @brief 决策树节点，包含左右子结点指针，自身id，包含的判断变量,判定条件
     * @todo 多个节点分配同一个函数的指针导致的问题（不同时调用应该问题不大）、所有变量定义为double类型！！！
     */
    // 判定函数1：决策变量>条件量
    bool compare_1(double a, double b);
    // 判定函数2：决策变量<条件量
    bool compare_2(double a, double b);
    // 判定函数3：决策变量==条件量
    bool compare_3(double a, double b);
    // 判定函数4：决策变量!=条件量
    bool compare_4(double a, double b);
    struct tree_node
    {
        int id;
        double *variable_ptr;
        // 定义函数指针类型
        using CompareFunction = bool (*)(double, double);
        // 函数指针成员
        CompareFunction compare_function;
        tree_node *left; // 左子节点指针
        int left_id;
        tree_node *right; // 右子节点指针
        int right_id;
        double condition_value;              // 条件量
        robot_msg::DecisionMsg decision_msg; // 决策信息
        ros::Publisher *decision_pub;        // 发布决策消息
        tree_node() : variable_ptr(nullptr), compare_function(nullptr), left(nullptr), right(nullptr) {}
        tree_node(double *x, CompareFunction func) : variable_ptr(x), compare_function(func), left(nullptr), right(nullptr) {}
        bool compare(double a, double b)
        {
            return compare_function(a, b);
        }
        void pub_decision()
        {
            decision_pub->publish(decision_msg);
        }
    };
    /**
     * @brief 生成决策图
     *
     */
    class decision_tree
    {
    public:
        // 调试
        std::chrono::time_point<std::chrono::system_clock> last_time;
        double a;
        double b;
        double c;
        int num;
        static void sighandler(int signum); // 设置退出函数
        decision_tree(const sp_decision::Blackboard::Ptr &blackboard_ptr, const tools::logger::Ptr &logger_ptr);
        ~decision_tree();
        void node_ptr_init();              // 生成决策图
        tree_node *judge(tree_node *node); // 执行判断
        void run();                        // 循环
        void print_tree();                 // 打印节点信息
        void run_start();
        std::vector<tree_node> nodes_vector; // 节点表
        std::vector<int> nodes_id_vector;    // 节点id表
        ros::Publisher decision_pub;

    private:
        ros::NodeHandle nh_;
        std::thread decision_thread_;
        bool decision_thread_running_;
        sp_decision::Blackboard::Ptr blackboard_ptr_;
        tools::logger::Ptr logger_ptr_;
        tools::yaml_reader::Ptr yaml_reader_ptr_;
    };
}
#endif
