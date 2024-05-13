/**
 * @file log.hpp
 * @author lp
 * @brief 记录日志，输出日志到/home/${USER}/log_decision/xxxx-xx-xx-xx-xx.txt
 * @version 0.1
 * @date 2024-04-06
 * @copyright Copyright (c) 2024
 */

#ifndef LOG_H
#define LOG_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <ctime>
#include <cstdlib>
namespace tools
{
    class logger
    {
    public:
        logger(int id)
        {
            char *username = getenv("USER"); // 获取用户名
            auto now = std::chrono::system_clock::now();
            // 将当前时间点转换为时间类型
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            // 将时间类型转换为struct tm类型，以便获取年、月、日、时、分、秒
            std::tm *now_tm = std::localtime(&now_time);
            std::stringstream ss;
            ss << "/home/" << username << "/log_decision/"<<id<<"/"<< (now_tm->tm_year + 1900)
               << "-" << (now_tm->tm_mon + 1) << "-" << now_tm->tm_mday << "-" << now_tm->tm_hour
               << "-" << now_tm->tm_min << "-" << now_tm->tm_sec << ".txt";
            std::string path = ss.str();
            file_logger_ = spdlog::basic_logger_mt("sp_decision", path);
            file_logger_->set_level(spdlog::level::debug);
        }
        void logInfo(const std::stringstream &message)
        {
            std::string log = message.str();
            file_logger_->info(log);
        }
        void logInfo(const std::string &msg)
        {
            file_logger_->info(msg);
        }
        void logDebug(const std::stringstream &message)
        {
            std::string log = message.str();
            file_logger_->debug(log);
        }
          void logDebug(const std::string &msg)
        {
            file_logger_->debug(msg);
        }
        void logError(const std::stringstream &message)
        {
            std::string log = message.str();
            file_logger_->error(log);
        }
          void logError(const std::string &msg)
        {
            file_logger_->error(msg);
        }
        typedef std::shared_ptr<logger> Ptr;
    private:
        std::shared_ptr<spdlog::logger> file_logger_;
    };
}

#endif // LOG_H