#ifndef __YOURS_DETECT_ARM_ERROR_CODE__
#define __YOURS_DETECT_ARM_ERROR_CODE__

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <yours_robot_tools/yours_log.h>
#include <stdlib.h>
namespace yours_detect_arm_error_code {

class YoursDetectArmErrorCode {
 private:
    /* data */
    ros::NodeHandle node_;
    ros::Subscriber error_sub_handle_;
    ros::Subscriber error_clean_handle_;
    void ErrorCodeCallback(const std_msgs::UInt32& msg) {
        // is_motor_error = false;
        if (msg.data != 0) {
            char data[50] = { 0 };
            sprintf(data, "0x%x", msg.data);
            log_->warn("Get ARM error code : " + std::string(data));
            if ((msg.data & 0x10000000) != 0) {
                if (((msg.data & 0x10000800) != 0) || ((msg.data & 0x10008000) != 0) || ((msg.data & 0x10000800) != 0) || ((msg.data & 0x10000080) != 0)) {
                    is_motor_error = true;
                    log_->error("Get Wheel error code : " + std::string(data) + ", robot stop.");
                }
            }
        }
        if (is_motor_error) {
            ROS_INFO_STREAM("stop robot");
        } else {
            // ROS_INFO_STREAM("run robot");
        }
    }

    void ErrorCleanCallback(const std_msgs::Bool& msg) {
        if (msg.data) {
            ClearError();
        }
    }
    std::shared_ptr<yours_log::YoursLog> log_;
    bool is_motor_error;

 public:
    YoursDetectArmErrorCode(/* args */ ros::NodeHandle& n)
        : node_(n)
        , is_motor_error(false) {
        std::string home_path(getenv("HOME"));
        std::cout << home_path << std::endl;
        home_path = home_path + "/log/yours_detect_arm_error_code";
        log_ = std::make_shared<yours_log::YoursLog>(home_path);
        log_->set_log_level(yours_log::debug);
        log_->set_print_level(yours_log::error);
        error_sub_handle_ = node_.subscribe("/yours_base/error_code", 10, &YoursDetectArmErrorCode::ErrorCodeCallback, this);
        error_clean_handle_ = node_.subscribe("/yours_clean_arm_error", 10, &YoursDetectArmErrorCode::ErrorCleanCallback, this);
    };
    ~YoursDetectArmErrorCode() {};
    inline bool IsMotorError() { return is_motor_error; };
    inline void ClearError() { is_motor_error = false; };
};

#if 0
YoursDetectArmErrorCode::YoursDetectArmErrorCode(/* args */)
{
}

YoursDetectArmErrorCode::~YoursDetectArmErrorCode()
{
}
#endif

}

#endif
