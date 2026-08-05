#include "../include/yours_navigation/yours_vision_detect_node.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "vision_test");
    ros::NodeHandle n("~");
    yours_vision_detect_node mVision(n);

    ros::Rate rate(10);
    while (ros::ok())
    {
        int pixel;
        if(mVision.run(pixel)){
            ROS_INFO_STREAM("have obstacles");
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}