/**
 * @file yaml_read.hpp
 * @author lp
 * @brief 解析yaml文件，并生成对应的决策森林
 * @version 0.1
 * @date 2024-04-06
 * @copyright Copyright (c) 2024
 */
#ifndef YAML_H
#define YAML_H

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
namespace tools
{
    class yaml_reader
    {
    public:
        yaml_reader(const std::string &filename) : filename_(filename) {}

        // 读取 YAML 文件并解析为 C++ 数据结构
        bool readYAML()
        {
            try
            {
                config_ = YAML::LoadFile(filename_);
                return true;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error reading YAML file: " << e.what() << std::endl;
                return false;
            }
        }

        // 获取解析后的 YAML 数据
        YAML::Node getConfig() const
        {
            return config_;
        }
        typedef std::shared_ptr<yaml_reader> Ptr;

    private:
        std::string filename_;
        YAML::Node config_;
    };
}

#endif // YAML_H

// #include "tools/yaml_read.hpp"
// struct DecisionTreeNode
// {
//     int id;
//     std::string variable;
//     std::string condition;
//     int left_id;
//     int right_id;
// };
// int main()
// {
//     // 创建 YAMLReader 实例并指定要读取的文件
//     YAMLReader reader("/home/lp1/sp_nav_ws/src/sp_nav/sp_decision/config/test.yaml");
//     double a = 0;
//     double *d;
//     std::vector<DecisionTreeNode> decisionTreeNodes;
//     // 读取并解析 YAML 文件
//     if (reader.readYAML())
//     {
//         // 获取解析后的 YAML 数据
//         YAML::Node config = reader.getConfig();
//         const YAML::Node &nodes = config["decision_tree_1"];
//         for (const auto &node : nodes)
//         {
//             DecisionTreeNode treeNode;
//             treeNode.id = node["id"].as<int>();
//             treeNode.variable = node["variable"].as<std::string>();
//             treeNode.condition = node["condition"].as<std::string>();
//             treeNode.left_id = node["left_id"].as<int>();
//             treeNode.right_id = node["right_id"].as<int>();
//             decisionTreeNodes.push_back(treeNode);
//         }
//         for (const auto &node : decisionTreeNodes)
//         {
//             std::cout << "ID: " << node.id << std::endl;
//             std::cout << "Variable: " << node.variable << std::endl;
//             std::cout << "Condition: " << node.condition << std::endl;
//             std::cout << "Left ID: " << node.left_id << std::endl;
//             std::cout << "Right ID: " << node.right_id << std::endl;
//             std::cout << std::endl;
//         }
//     }
//     return 0;
// }
