#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <cmath>
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

#include <cstring>
#include <netinet/in.h>
#include <thread>
namespace tools
{
    // 将弧度值限制在(-pi, pi]
    inline double limit_rad(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle <= -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    // 四元数转欧拉角
    // x = 0, y = 1, z = 2
    // e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
    // 参考：https://github.com/evbernardes/quaternion_to_euler
    inline Eigen::Vector3d eulers(
        Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false)
    {
        if (!extrinsic)
            std::swap(axis0, axis2);

        auto i = axis0, j = axis1, k = axis2;
        auto is_proper = (i == k);
        if (is_proper)
            k = 3 - i - j;
        auto sign = (i - j) * (j - k) * (k - i) / 2;

        double a, b, c, d;
        Eigen::Vector4d xyzw = q.coeffs();
        if (is_proper)
        {
            a = xyzw[3];
            b = xyzw[i];
            c = xyzw[j];
            d = xyzw[k] * sign;
        }
        else
        {
            a = xyzw[3] - xyzw[j];
            b = xyzw[i] + xyzw[k] * sign;
            c = xyzw[j] + xyzw[3];
            d = xyzw[k] * sign - xyzw[i];
        }

        Eigen::Vector3d eulers;
        auto n2 = a * a + b * b + c * c + d * d;
        eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

        auto half_sum = std::atan2(b, a);
        auto half_diff = std::atan2(-d, c);

        auto eps = 1e-7;
        auto safe1 = std::abs(eulers[1]) >= eps;
        auto safe2 = std::abs(eulers[1] - M_PI) >= eps;
        auto safe = safe1 && safe2;
        if (safe)
        {
            eulers[0] = half_sum + half_diff;
            eulers[2] = half_sum - half_diff;
        }
        else
        {
            if (!extrinsic)
            {
                eulers[0] = 0;
                if (!safe1)
                    eulers[2] = 2 * half_sum;
                if (!safe2)
                    eulers[2] = -2 * half_diff;
            }
            else
            {
                eulers[2] = 0;
                if (!safe1)
                    eulers[0] = 2 * half_sum;
                if (!safe2)
                    eulers[0] = 2 * half_diff;
            }
        }

        for (int i = 0; i < 3; i++)
            eulers[i] = limit_rad(eulers[i]);

        if (!is_proper)
        {
            eulers[2] *= sign;
            eulers[1] -= M_PI / 2;
        }

        if (!extrinsic)
            std::swap(eulers[0], eulers[2]);

        return eulers;
    }

    // 旋转矩阵转欧拉角
    // x = 0, y = 1, z = 2
    // e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
    inline Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false)
    {
        Eigen::Quaterniond q(R);
        return eulers(q, axis0, axis1, axis2, extrinsic);
    }

    // 四元数转旋转矩阵
    inline Eigen::Matrix3d quaterniond2matrix(const geometry_msgs::TransformStamped &transform)
    {
        // 定义一个四元数对象
        Eigen::Quaterniond quaternion(transform.transform.rotation.w, transform.transform.rotation.x,
                                      transform.transform.rotation.y, transform.transform.rotation.z); // (w, x, y, z)
        // 将四元数转换为旋转矩阵
        Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
        return rotation_matrix;
    }
    inline geometry_msgs::PoseStamped trans(const geometry_msgs::PoseStamped &t_in,
                                            const geometry_msgs::TransformStamped &transform)
    {
        geometry_msgs::PoseStamped out;
        Eigen::Vector3d pose(t_in.pose.position.x, t_in.pose.position.y, t_in.pose.position.z);
        Eigen::Vector3d vec(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        Eigen::Vector3d out_vec = quaterniond2matrix(transform) * pose + vec;
        out.pose.position.x = out_vec[0];
        out.pose.position.y = out_vec[1];
        out.pose.position.z = out_vec[2];
        return out;
    }
} // namespace tools

#endif // TOOLS__MATH_TOOLS_HPP