#include <ros/ros.h>
#include "../include/yours_navigation/yourslidardetect.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yours_lidar_detect");
    ros::NodeHandle n("~");
    ros::Rate rate(10);
    YoursLidarDetect detect(n);
    int lidarDetectCode;
    std_msgs::Float32MultiArray lidarDetectDataArray;
    while (ros::ok())
    {
        detect.run(lidarDetectCode, lidarDetectDataArray);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}