#include "decision_node/decision_tree.hpp"
namespace sp_decision
{
    // 判定函数0：返回true，确保动作节点下一个一定是左节点
    bool compare_0(double a, double b)
    {
        return true;
    }
    // 判定函数1：决策变量>条件量
    bool compare_1(double a, double b)
    {
        return (a > b) ? 1 : 0;
    }
    // 判定函数2：决策变量<条件量
    bool compare_2(double a, double b)
    {
        return (a < b) ? 1 : 0;
    }
    // 判定函数3：决策变量==条件量
    bool compare_3(double a, double b)
    {
        return (a == b) ? 1 : 0;
    }
    // 判定函数4：决策变量!=条件量
    bool compare_4(double a, double b)
    {
        return (a != b) ? 1 : 0;
    }
    void decision_tree::sighandler(int signum)
    {
        printf("用户退出...\n");
    }
    decision_tree::decision_tree(const sp_decision::Blackboard::Ptr &blackboard_ptr, const tools::logger::Ptr &logger_ptr)
    {
        decision_pub =
            nh_.advertise<robot_msg::DecisionMsg>("/sentry/decision", 1);
        last_time = std::chrono::system_clock::now();
        a = 2;
        b = 3;
        c = 0;
        num = 0;
        yaml_reader_ptr_ = std::make_shared<tools::yaml_reader>(ros::package::getPath("sp_decision") + "/config/testv3.yaml");
        blackboard_ptr_ = blackboard_ptr;
        logger_ptr_ = logger_ptr;
        node_ptr_init(); // 生成决策图
    }
    decision_tree::~decision_tree()
    {
        if (decision_thread_.joinable())
        {
            decision_thread_running_ = false;
            decision_thread_.join();
        }
        // delete root_node_;
    }
    void decision_tree::print_tree()
    {
        for (auto &node : nodes_vector)
        {
            std::cout << "ID: " << node.id << std::endl;
            std::cout << "Left ID: " << node.left->id << std::endl;
            std::cout << "Right ID: " << node.right->id << std::endl;
            std::cout << "condition_value: " << node.condition_value << std::endl;
            std::cout << "variable_value: " << *node.variable_ptr << std::endl;
            std::cout << "action: " << node.decision_msg.decision << std::endl;
            std::cout << std::endl;
        }
    }
    void decision_tree::node_ptr_init()
    {
        if (yaml_reader_ptr_->readYAML())
        {
            YAML::Node config = yaml_reader_ptr_->getConfig();
            for (const auto &node : config["decision_tree_1"])
            {
                tree_node dt_node;
                std::string symbol; // 获取符号
                double value;
                dt_node.id = node["id"].as<int>();
                dt_node.left_id = node["left_id"].as<int>();
                dt_node.right_id = node["right_id"].as<int>();
                std::string variable = node["variable"].as<std::string>();
                std::string condition = node["condition"].as<std::string>();
                std::string action = node["action"].as<std::string>();
                if (action != "") // 动作节点
                {
                    size_t pos = action.find_first_of("0123456789");
                    int type = std::stoi(action.substr(pos, 1));
                    std::string remainingString = action.substr(pos + 1);
                    dt_node.decision_pub = &decision_pub;
                    dt_node.decision_msg.decision = remainingString;
                    dt_node.decision_msg.type = type;
                    dt_node.compare_function = compare_0;
                    dt_node.variable_ptr = &blackboard_ptr_->match_progress;
                    dt_node.condition_value = 0;
                }
                else // 判断节点
                {
                    size_t pos = 0;
                    while (pos < condition.size() && condition[pos] == ' ')
                        ++pos;
                    // 获取符号
                    while (pos < condition.size() && !isdigit(condition[pos]) && condition[pos] != ' ')
                        symbol += condition[pos++];
                    if (symbol == ">")
                    {
                        dt_node.compare_function = compare_1;
                    }
                    if (symbol == "<")
                    {
                        dt_node.compare_function = compare_2;
                    }
                    if (symbol == "==")
                    {
                        dt_node.compare_function = compare_3;
                    }
                    if (symbol == "!=")
                    {
                        dt_node.compare_function = compare_4;
                    }
                    // 获取数字部分
                    value = std::stod(condition.substr(pos));
                    dt_node.condition_value = value;
                    // 获取变量名
                    if (variable == "symbol")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->action_symbol;
                    }
                    if (variable == "match_progress")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->match_progress;
                    }
                    if (variable == "match_remainder")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->match_remainder;
                    }
                    if (variable == "sentry_hp")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->sentry_status.sentry_hp;
                    }
                    if (variable == "key")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->key_from_char;
                    }
                    if (variable == "target_update")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->target_update;
                    }
                    if (variable == "outpost")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->team_hp[6];
                    }
                    if (variable == "sentry_bullet")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->sentry_bullet;
                    }
                    if (variable == "money")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->money;
                    }          
                    if (variable == "buy_bullet_remain_time")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->buy_bullet_remain_time;
                    }    
                    if (variable == "on_addblood_area")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->on_addblood_area;
                    }   
                    if (variable == "sentry_attackable")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->sentry_attackable;
                    }   
                    if (variable == "buy_bullet_wait_time")
                    {
                        dt_node.variable_ptr = &blackboard_ptr_->buy_bullet_wait_time;
                    }   
                }
                nodes_id_vector.push_back(dt_node.id);
                nodes_vector.push_back(dt_node);
            }
            // 连接节点
            for (auto &node : nodes_vector)
            {
                if (node.left_id != -1)
                {
                    int m = 0;
                    for (m; m < nodes_id_vector.size(); m++)
                    {
                        if (nodes_id_vector[m] == node.left_id)
                            break;
                    }
                    node.left = &nodes_vector[m];
                }
                else
                    node.left = &nodes_vector[0];
                if (node.right_id != -1)
                {
                    int m = 0;
                    for (m; m < nodes_id_vector.size(); m++)
                    {
                        if (nodes_id_vector[m] == node.right_id)
                            break;
                    }
                    node.right = &nodes_vector[m];
                }
                else
                    node.right = &nodes_vector[0];
            }
        }
    }
    tree_node *decision_tree::judge(tree_node *node)
    {
        std::stringstream log_msg;
        log_msg << "id:" << node->id;
        std::cout<<node->id<<std::endl;
        logger_ptr_->logInfo(log_msg);
        if (node->id == 1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 间隔50ms
            num++;
            //std::cout << "num:" << num << std::endl;
            // auto now = std::chrono::system_clock::now();
            // std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time);
            // std::cout << "delat-time" << duration.count() << std::endl;
            // last_time = std::chrono::system_clock::now();
        }
        if (node->compare(*node->variable_ptr, node->condition_value))
        {
            if (node->left->id != 1)
            {
                //std::cout << "judge_true\t" << node->id << std::endl;
                // std::cout << "variable\t" << *node->variable_ptr << std::endl;
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                return judge(node->left);
            }
            else
            {
                node->compare(*node->variable_ptr, node->condition_value);
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                //std::cout << "judge_true\t" << node->id << std::endl;
                // std::cout << "variable\t" << *node->variable_ptr << std::endl;
            }
        }
        else
        {
            if (node->right->id != 1)
            {
                //std::cout << "judge_false\t" << node->id << std::endl;
                // std::cout << "variable\t" << *node->variable_ptr << std::endl;
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                return judge(node->right);
            }
            else
            {
                node->compare(*node->variable_ptr, node->condition_value);
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                //std::cout << "judge_true\t" << node->id << std::endl;
                // std::cout << "variable\t" << *node->variable_ptr << std::endl;
            }
        }
    }
    void decision_tree::run()
    {
        // signal(SIGINT, sighandler);//强制退出程序
        while (decision_thread_running_)
        {
            judge(&nodes_vector[0]);
        }
    }
    void decision_tree::run_start()
    {
        decision_thread_running_ = true;
        decision_thread_ = std::thread(&decision_tree::run, this);
    }
}
