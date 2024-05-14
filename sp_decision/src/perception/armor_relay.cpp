/**
 * @file armor_relay.cpp
 * @author lp
 * @todo 接收并处理所有坐标信息
 * @version 0.1
 * @date 2024-04-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "perception/armor_relay.hpp"
#include "tools/math.hpp"
std::string enemy_string;
void EnemyCallback(const robot_msg::EnemyStage::ConstPtr &msg)
{
    enemy_string = msg->ss;
}
int main(int argc, char **argv)
{
    enemy_string = "0";

    ros::init(argc, argv, "armor_transformer");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher armor_pub_ = nh.advertise<robot_msg::Armor>("/armor", 1);
    ros::Subscriber enemy_sub_ = nh.subscribe("/enemy_stage", 1, &EnemyCallback);
    std::vector<robot_msg::Armor> enemy_pos;
    robot_msg::Armor armor_sentry;
    armor_sentry.number = 7;
    armor_sentry.type = "sentry";

    geometry_msgs::PoseStamped src_pose;
    src_pose.header.frame_id = "world_in_auto_aim"; // 源坐标系
    src_pose.pose.orientation.x = 0.0;
    src_pose.pose.orientation.y = 0.0;
    src_pose.pose.orientation.z = 0.0;
    src_pose.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped dst_pose;

    TCPServer server;
    if (!server.start())
    {
        std::cerr << "启动服务器失败" << std::endl;
        return 1;
    }
    while (1)
    { // 接受客户端连接
        if (!server.acceptConnection())
            break;
        while (1)
        {
            std::string receivedData = server.receive();
            std::vector<std::string> splittedData = server.splitString(receivedData, ',');
            std::cout << "Received data: ";
            std::vector<double> data;
            for (const auto &str : splittedData)
            {
                data.push_back(std::stod(str)); // 分别为跟踪状态，x,y,z坐标
            }
            armor_sentry.track_status = data[0];
            src_pose.pose.position.x = data[1];
            src_pose.pose.position.y = data[2];
            src_pose.pose.position.z = data[3];
            ROS_INFO(" pose: x=%f, y=%f, z=%f",
                     src_pose.pose.position.x,
                     src_pose.pose.position.y,
                     src_pose.pose.position.z);
            // if (data.size() == 4)
            //     break;
            try
            {
                geometry_msgs::TransformStamped body2camera_init;
                body2camera_init = tfBuffer.lookupTransform("camera_init", "body", ros::Time(0));
                body2camera_init.transform.rotation.w = 1;
                body2camera_init.transform.rotation.x = 0;
                body2camera_init.transform.rotation.y = 0;
                body2camera_init.transform.rotation.z = 0;
                geometry_msgs::PoseStamped pose_1 = tools::trans(src_pose, body2camera_init);
                geometry_msgs::TransformStamped camera_init2map;
                camera_init2map = tfBuffer.lookupTransform("map", "camera_init", ros::Time(0));
                geometry_msgs::PoseStamped pose_2 = tools::trans(pose_1, camera_init2map);
                //  输出转换后的姿态
                armor_sentry.pose.position.x = pose_2.pose.position.x;
                armor_sentry.pose.position.y = pose_2.pose.position.y;
                armor_sentry.pose.position.z = pose_2.pose.position.z;
                ROS_INFO("Transformed pose: x=%f, y=%f, z=%f",
                         armor_sentry.pose.position.x,
                         armor_sentry.pose.position.y,
                         armor_sentry.pose.position.z);
                armor_pub_.publish(armor_sentry);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Failed to transform pose: %s", ex.what());
            }
            // 处理回调函数
            ros::spinOnce();
            server.send(enemy_string);
        }
    }
    return 0;
}
