/**
 * @file armor_relay.hpp
 * @author lp
 * @brief 转发自瞄发送的装甲板坐标信息，经过一次坐标变换到map坐标系，同时在接收到客户端发送的消息后返回一次消息
 * @version 0.1
 * @date 2024-04-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ARMOR_RELAY_H
#define ARMOR_RELAY_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <vector>
#include <sstream>
#include "robot_msg/Armor.h"
#include "robot_msg/EnemyStage.h"
#include <cstring>
#include <netinet/in.h>
#include <thread>
#define recvBuf_Size 1024

class TCPServer
{
private:
    int listenfd;
    int sock; // Socket for the connected client

public:
    TCPServer() : listenfd(-1), sock(-1) {}

    ~TCPServer()
    {
        close(listenfd);
    }

    bool start()
    {
        // 创建一个监听socket
        listenfd = socket(AF_INET, SOCK_STREAM, 0);
        if (listenfd == -1)
        {
            std::cerr << "创建监听套接字失败" << std::endl;
            return false;
        }

        // 初始化服务器地址
        struct sockaddr_in bindaddr;
        bindaddr.sin_family = AF_INET;
        bindaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        bindaddr.sin_port = htons(12345);

        // 将socket绑定到服务器地址
        if (bind(listenfd, reinterpret_cast<struct sockaddr *>(&bindaddr), sizeof(bindaddr)) == -1)
        {
            std::cerr << "绑定套接字失败" << std::endl;
            close(listenfd);
            return false;
        }

        // 开始监听客户端连接
        if (listen(listenfd, SOMAXCONN) == -1)
        {
            std::cerr << "监听套接字失败" << std::endl;
            close(listenfd);
            return false;
        }

        return true;
    }

    bool acceptConnection()
    {
        struct sockaddr_in clientaddr;
        ROS_INFO("Waiting for connecting");
        socklen_t clientaddrlen = sizeof(clientaddr);

        // 接受客户端的连接
        sock = accept(listenfd, reinterpret_cast<struct sockaddr *>(&clientaddr), &clientaddrlen);
        if (sock == -1)
        {
            std::cerr << "接受客户端连接失败" << std::endl;
            return false;
        }
        return true;
    }

    std::string receive()
    {
        char buffer[recvBuf_Size] = {0};
        std::string receivedData;
        int bytesReceived = recv(sock, buffer, sizeof(buffer), 0);
        if (bytesReceived < 0)
        {
            std::cerr << "接收数据失败" << std::endl;
        }
        else
        {
            receivedData = std::string(buffer, bytesReceived);
        }
        return receivedData;
    }

    std::vector<std::string> splitString(const std::string &input, char delimiter)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(input);
        std::string token;
        while (std::getline(ss, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }
    bool send(const std::string &data)
    {
        // 检查套接字是否有效
        if (sock == -1)
        {
            std::cerr << "套接字未连接" << std::endl;
            return false;
        }
        // 发送数据
        int bytesSent = ::send(sock, data.c_str(), data.size(), 0);
        if (bytesSent < 0)
        {
            std::cerr << "发送数据失败" << std::endl;
            return false;
        }
        return true;
    }
};

#endif