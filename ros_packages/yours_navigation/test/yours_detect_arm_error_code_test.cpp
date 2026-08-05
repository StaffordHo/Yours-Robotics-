#include "../include/yours_navigation/yours_detect_arm_error_code.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle n;
    std::shared_ptr<yours_detect_arm_error_code::YoursDetectArmErrorCode>
        error_code(new yours_detect_arm_error_code::YoursDetectArmErrorCode(n));

    ros::Rate rate(10);
    while (ros::ok()) {
        /* code */
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
