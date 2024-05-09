#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "tools/math.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "robot_msg/CmdGimbal.h"
#include <geometry_msgs/PoseStamped.h>
cv::Mat vel_img;
ros::Publisher sentry_cmd_vel_pub_;
ros::Publisher sentry_yaw_pub_;
ros::Publisher sentry_odo_pub_;
ros::Publisher pubOdomAftMapped;
ros::Time last_time;
nav_msgs::Odometry odo_receive;
nav_msgs::Odometry odo_publish;
bool odo_received;
void get_cmd_vel(const geometry_msgs::Twist &msg)
{
    ROS_INFO("linear_x: %f", msg.linear.x);
    ROS_INFO("linear_y: %f", msg.linear.y);
    ROS_INFO("angular_z: %f", msg.angular.z);
    geometry_msgs::Twist vel;
    vel.linear.x = msg.linear.x;
    vel.linear.y = msg.linear.y;
    vel.angular.z = -4;
    sentry_cmd_vel_pub_.publish(vel);
    // 计算速度矢量的大小
    double magnitude = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2));

    // 计算速度矢量的方向（以弧度表示）
    double direction = atan2(msg.linear.y, msg.linear.x);

    // 将方向从弧度转换为度
    direction *= 180.0 / M_PI;

    // 在 vel_img 上绘制速度矢量
    int center_x = vel_img.cols / 2;
    int center_y = vel_img.rows / 2;
    int arrow_length = 150;

    int end_x = center_x - arrow_length * (msg.linear.y);
    int end_y = center_y - arrow_length * (msg.linear.x);

    // cv::arrowedLine(vel_img, cv::Point(center_x, center_y), cv::Point(end_x, end_y), cv::Scalar(255), 2);
    // cv::imshow("Velocity Image", vel_img);
    // cv::waitKey(1);
    // vel_img.setTo(cv::Scalar(0));
    // tf::TransformBroadcaster odom_broadcaster;

    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = ros::Time::now();
    // odom_trans.header.frame_id = "body";
    // odom_trans.child_frame_id = "vel";
    // odom_trans.transform.translation.x = x_pos;
    // odom_trans.transform.translation.y = y_pos;
    // odom_trans.transform.translation.z = 0.0;
    // double theta = atan(msg.linear.y / msg.linear.x);
    // if (theta < 0)
    //     theta *= -1;
    // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

    // // 发送变换
    // odom_broadcaster.sendTransform(odom_trans);

    // // 发布里程计消息
    // nav_msgs::Odometry odom;
    // odom.header.stamp = odom_trans.header.stamp;
    // odom.header.frame_id = "body";
    // odom.child_frame_id = "vel";

    // odom.pose.pose.position.x = x_pos;
    // odom.pose.pose.position.y = y_pos;
    // odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = odom_trans.transform.rotation;

    // odom_pub.publish(odom);
}
void get_odo(const nav_msgs::Odometry::ConstPtr msg)
{
    odo_received = true;
    odo_receive = *msg;
    last_time = ros::Time::now();
}
void odo_pub()
{
    // if (odo_received)
    // {
    //     pubOdomAftMapped.publish(odo_receive);
    //     odo_received = false;
    // }
    // else
    // {
        odo_publish = odo_receive;
        Eigen::Quaterniond q(odo_receive.pose.pose.orientation.w, odo_receive.pose.pose.orientation.x, odo_receive.pose.pose.orientation.y, odo_receive.pose.pose.orientation.z);
        Eigen::Vector3d euler = tools::eulers(q, 2, 1, 0);
        double yaw = euler[0];
        double w = odo_receive.twist.twist.angular.z;

        yaw += 10*w * (ros::Time::now() - last_time).toSec();
        yaw=tools::limit_rad(yaw);
        ROS_INFO("w-----------:%f",w * (ros::Time::now() - last_time).toSec());
        ROS_INFO("yaw-----------:%f", yaw);
        // 根据欧拉角构造旋转矩阵
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        // 将旋转矩阵转换为四元数
        Eigen::Quaterniond quaternion(rotation_matrix);
        odo_publish.pose.pose.orientation.w = quaternion.w();
        odo_publish.pose.pose.orientation.x = quaternion.x();
        odo_publish.pose.pose.orientation.y = quaternion.y();
        odo_publish.pose.pose.orientation.z = quaternion.z();
        pubOdomAftMapped.publish(odo_publish);
    //}
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_test");
    ros::NodeHandle nh("~");
    vel_img = cv::Mat(400, 400, CV_8UC1, cv::Scalar(0));
    ros::Subscriber enemy_sub_ = nh.subscribe("/cmd_vel", 1, get_cmd_vel);
    ros::Subscriber odo = nh.subscribe("/Odometry1", 1, get_odo);
    sentry_cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/sentry/cmd_vel", 1);
    sentry_yaw_pub_ = nh.advertise<robot_msg::CmdGimbal>("/sentry/cmd_gimbal", 1);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Rate rate(100); // 设置发布频率为10Hz
    // while (ros::ok)
    // {
    //     odo_pub();
    //     ros::spinOnce();
    //     rate.sleep();
    //     // ros::Duration(0.01).sleep();
    // }
     ros::spin();
}
