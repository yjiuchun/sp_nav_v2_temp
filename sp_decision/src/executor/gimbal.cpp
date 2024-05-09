#include "executor/gimbal.hpp"
namespace sp_decision
{
    GimbalExecutor::GimbalExecutor(const tools::logger::Ptr &logger_ptr)
    {
        logger_ptr_ = logger_ptr; // 获取日志器
        gimbal_pub_ = nh_.advertise<robot_msg::CmdGimbal>("/sentry/cmd_gimbal", 1);
    }
    /**
     * @brief 设置云台运动模式
     *
     * @param min_angle 向右最大角度
     * @param max_angle 向左最大角度
     * @param pitch_enable 是否允许俯仰
     * @todo 电控增加pitch轴运动
     */
    void GimbalExecutor::gimbal_set(double min_angle, double max_angle, bool pitch_enable = false)
    {
        robot_msg::CmdGimbal gimbal;
        gimbal.yaw_min = min_angle;
        gimbal.yaw_max = max_angle;
        gimbal.pitch_max = 30;
        gimbal.pitch_min = -10;
        if (pitch_enable)
        {
            gimbal.pitch_max = 0;
            gimbal.pitch_min = 0;
        }
        gimbal_pub_.publish(gimbal);
    }
}
