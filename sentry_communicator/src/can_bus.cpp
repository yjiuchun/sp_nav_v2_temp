#include "sentry_communicator/can_bus.hpp"
#define MAX_SPEED 10.f
#define MIN_SPEED -10.f
#define MAX_ANGLE 180.f
#define MIN_ANGLE -180.f

// #define DEBUG // 调试时打开,防止和虚拟裁判系统冲突

namespace sentry_communicator
{

    CanBus::CanBus(const std::string &bus_name, int thread_priority, ros::NodeHandle &root_nh)
        : bus_name_(bus_name)
    {
        referee_info_pub_ = root_nh.advertise<robot_msg::RefereeInfoMsg>("referee_info",1000);
        robot_EnemyHP_pub_ = root_nh.advertise<robot_msg::RobotHP>("Enemy_robot_HP",1000);
        robot_TeamHP_pub_ = root_nh.advertise<robot_msg::RobotHP>("Team_robot_HP",1000);
        robot_shoot_pub_ = root_nh.advertise<robot_msg::shoot>("robot_shoot",1000);

        while (!socket_can_.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) && ros::ok())
            ros::Duration(.5).sleep();
        ROS_INFO("[CAN_BUS] : Successfully connected to %s.", bus_name.c_str());
        
        // subscribe cmd_vel
        cmd_chassis_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/sentry/cmd_vel", 1, &CanBus::cmdChassisCallback, this);
        //cmd_chassis_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &CanBus::cmdChassisCallback, this);
        chassis_frame_.can_id = 0x111;
        chassis_frame_.can_dlc = 8;

        // subscribe gimble yaw limit
        cmd_gimbal_sub_ = root_nh.subscribe<robot_msg::CmdGimbal>("/sentry/cmd_gimbal", 1, &CanBus::cmdGimbalCallback, this);
        gimbal_frame_.can_id = 0x112;
        gimbal_frame_.can_dlc = 8;
    }

    void CanBus::write()
    {
        // set all data_byte to 0
        std::fill(std::begin(chassis_frame_.data), std::end(chassis_frame_.data), 0);
        
        const geometry_msgs::Twist &command_velocity = chassis_buffer_.readFromNonRT()->cmd_vel_;

        uint16_t vel_x = float2uint(command_velocity.linear.x, MIN_SPEED, MAX_SPEED, 12);
        uint16_t vel_y = float2uint(command_velocity.linear.y, MIN_SPEED, MAX_SPEED, 12);
        uint16_t vel_z = float2uint(command_velocity.angular.z, MIN_SPEED, MAX_SPEED, 12);
        uint16_t buy_bullet = command_velocity.linear.z;

        chassis_frame_.data[0] = static_cast<uint8_t>(vel_x >> 4u);
        chassis_frame_.data[1] = static_cast<uint8_t>((vel_x & 0xF) << 4u | vel_y >> 8u);
        chassis_frame_.data[2] = static_cast<uint8_t>(vel_y);
        chassis_frame_.data[3] = static_cast<uint8_t>(vel_z >> 4u);
        chassis_frame_.data[4] = static_cast<uint8_t>((vel_z & 0xF) << 4u | 0xF);
        chassis_frame_.data[5] = static_cast<uint8_t>(buy_bullet >> 8);
        chassis_frame_.data[6] = static_cast<uint8_t>(buy_bullet);

    	socket_can_.write(&chassis_frame_);


        // set all data_byte to 0
        std::fill(std::begin(gimbal_frame_.data), std::end(gimbal_frame_.data), 0);
        
        const robot_msg::CmdGimbal &gimbal_command = gimbal_buffer_.readFromNonRT()->cmd_gimbal_;

        uint16_t yaw_lower_limit = float2uint(gimbal_command.yaw_min, MIN_ANGLE, MAX_ANGLE, 16);
        uint16_t yaw_upper_limit = float2uint(gimbal_command.yaw_max, MIN_ANGLE, MAX_ANGLE, 16);
        uint16_t pitch_lower_limit = float2uint(gimbal_command.pitch_min, MIN_ANGLE, MAX_ANGLE, 16);
        uint16_t pitch_upper_limit = float2uint(gimbal_command.pitch_max, MIN_ANGLE, MAX_ANGLE, 16);

        gimbal_frame_.data[0] = static_cast<uint8_t>(yaw_lower_limit >> 8);
        gimbal_frame_.data[1] = static_cast<uint8_t>(yaw_lower_limit);
        gimbal_frame_.data[2] = static_cast<uint8_t>(yaw_upper_limit >> 8);
        gimbal_frame_.data[3] = static_cast<uint8_t>(yaw_upper_limit);
        gimbal_frame_.data[4] = static_cast<uint8_t>(pitch_lower_limit >> 8);
        gimbal_frame_.data[5] = static_cast<uint8_t>(pitch_lower_limit);
        gimbal_frame_.data[6] = static_cast<uint8_t>(pitch_upper_limit >> 8);
        gimbal_frame_.data[7] = static_cast<uint8_t>(pitch_upper_limit);

    	socket_can_.write(&gimbal_frame_);
    }

    void CanBus::cmdChassisCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        ChassisCommand cmd_struct_temp{.cmd_vel_ = *msg, .stamp_ = ros::Time::now()};
        chassis_buffer_.writeFromNonRT(cmd_struct_temp);
    }

    void CanBus::cmdGimbalCallback(const robot_msg::CmdGimbal::ConstPtr &msg)
    {
        GimbalCommand cmd_struct_temp{.cmd_gimbal_ = *msg, .stamp_ = ros::Time::now()};
        gimbal_buffer_.writeFromNonRT(cmd_struct_temp);
    }

    void CanBus::frameCallback(const can_frame &frame)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (frame.can_id == 0x18A)
        {
            robot_TeamHP_msg_.Hero_HP = 
                (uint16_t)((frame.data[0] << 8u) | frame.data[1]);
            robot_TeamHP_msg_.Engineer_HP = 
                (uint16_t)((frame.data[2] << 8u) | frame.data[3]);
            robot_TeamHP_msg_.Infantry_3_HP =
                (uint16_t)((frame.data[4] << 8u) | frame.data[5]);
            robot_TeamHP_msg_.Infantry_4_HP =
                (uint16_t)((frame.data[6] << 8u) | frame.data[7]);
            if (lower_Teamdata_updated_ == true && upper_Teamdata_updated_ == false)
            {
                // 发布数据
#ifndef DEBUG
                robot_TeamHP_pub_.publish(robot_TeamHP_msg_);
#endif 
                lower_Teamdata_updated_ = false;
                upper_Teamdata_updated_ = false;
            }
            else 
            {
                upper_Teamdata_updated_ = true;
            }
        }

        if (frame.can_id == 0x18B)
        {
            robot_TeamHP_msg_.Infantry_5_HP = 
                (uint16_t)((frame.data[0] << 8u) | frame.data[1]);
            robot_TeamHP_msg_.Sentry_HP = 
                (uint16_t)((frame.data[2] << 8u) | frame.data[3]);
            robot_TeamHP_msg_.OutPose_HP =
                (uint16_t)((frame.data[4] << 8u) | frame.data[5]);
            robot_TeamHP_msg_.Base_HP =
                (uint16_t)((frame.data[6] << 8u) | frame.data[7]);
            if (upper_Teamdata_updated_ == true && lower_Teamdata_updated_ == false)
            {
                // 发布数据
#ifndef DEBUG
                robot_TeamHP_pub_.publish(robot_TeamHP_msg_);
#endif 
                lower_Teamdata_updated_ = false;
                upper_Teamdata_updated_ = false;
            }
            else 
            {
                lower_Teamdata_updated_ = true;
            }
        }
        if (frame.can_id == 0x18C)
        {
            robot_EnemyHP_msg_.Hero_HP = 
                (uint16_t)((frame.data[0] << 8u) | frame.data[1]);
            robot_EnemyHP_msg_.Engineer_HP = 
                (uint16_t)((frame.data[2] << 8u) | frame.data[3]);
            robot_EnemyHP_msg_.Infantry_3_HP =
                (uint16_t)((frame.data[4] << 8u) | frame.data[5]);
            robot_EnemyHP_msg_.Infantry_4_HP =
                (uint16_t)((frame.data[6] << 8u) | frame.data[7]);
            if (lower_Enemydata_updated_ == true && upper_Enemydata_updated_ == false)
            {
                // 发布数据
#ifndef DEBUG
                robot_EnemyHP_pub_.publish(robot_EnemyHP_msg_);
#endif 
                lower_Enemydata_updated_ = false;
                upper_Enemydata_updated_ = false;
            }
            else 
            {
                upper_Enemydata_updated_ = true;
            }
        }

        if (frame.can_id == 0x18D)
        {
            robot_EnemyHP_msg_.Infantry_5_HP = 
                (uint16_t)((frame.data[0] << 8u) | frame.data[1]);
            robot_EnemyHP_msg_.Sentry_HP = 
                (uint16_t)((frame.data[2] << 8u) | frame.data[3]);
            robot_EnemyHP_msg_.OutPose_HP =
                (uint16_t)((frame.data[4] << 8u) | frame.data[5]);
            robot_EnemyHP_msg_.Base_HP =
                (uint16_t)((frame.data[6] << 8u) | frame.data[7]);
            if (upper_Enemydata_updated_ == true && lower_Enemydata_updated_ == false)
            {
                // 发布数据
#ifndef DEBUG
                robot_EnemyHP_pub_.publish(robot_EnemyHP_msg_);
#endif 
                lower_Enemydata_updated_ = false;
                upper_Enemydata_updated_ = false;
            }
            else 
            {
                lower_Enemydata_updated_ = true;
            }
        }
        
        if(frame.can_id == 0x18F){

            referee_info_msg_.game_progress = frame.data[0];
            referee_info_msg_.stage_remain_time = (uint16_t)((frame.data[1] << 8u) | frame.data[2]);
            
            data = (frame.data[3] << 8u) | frame.data[4];
            referee_info_msg_.target_x = uint2float(data, -30, 30, 16);
            data = (frame.data[5] << 8u) | frame.data[6];
            referee_info_msg_.target_y = uint2float(data, -30, 30, 16);

            referee_info_msg_.key = (char)frame.data[7];
#ifndef DEBUG
            referee_info_pub_.publish(referee_info_msg_);
#endif 
        }
        if(frame.can_id == 0x18E){

            robot_shoot_msg_.shoot_power = frame.data[0];
            robot_shoot_msg_.allowance_17mm = (frame.data[1] << 8u | frame.data[2]);
            robot_shoot_msg_.remain_gold = (frame.data[3] << 8u | frame.data[4]);
            robot_shoot_msg_.redeemed_17mm = (frame.data[5] << 8u | frame.data[6]);
#ifndef DEBUG
            robot_shoot_pub_.publish(robot_shoot_msg_);
#endif 
            // Robot_ID = frame.data[0];
            // Keyboard = frame.data[1];
            // data = (frame.data[2] << 8u) | frame.data[3];
            // float vel_x = uint2float(data, -30, 30, 16);
            // data = (frame.data[4] << 8u) | frame.data[5];
            // float vel_y = uint2float(data, -30, 30, 16);
            // lower_com_data.x = vel_x;
            // lower_com_data.y = vel_y;
            // lower_com_data.z = Keyboard;
            // lowercom_data_pub.publish(lower_com_data);
        }

    }

} // namespace : sentry_communicator