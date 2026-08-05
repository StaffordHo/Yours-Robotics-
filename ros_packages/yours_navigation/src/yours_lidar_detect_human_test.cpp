#include "../include/yours_navigation/yours_lidar_detect_human.h"
//#include <yours_robot_tools/yours_log.h>
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "yours_lidar_test");

    ros::NodeHandle n("~");
    //std::shared_ptr<yours_log::YoursLog> log(new yours_log::YoursLog("/home/jjiangbo/log/yours_lidar_test/"));
    //std::shared_ptr<yours_lidar_detect_human::YoursLidarDetectHuman> lidar(new yours_lidar_detect_human::YoursLidarDetectHuman(n, log));
    std::shared_ptr<yours_lidar_detect_human::YoursLidarDetectHuman> lidar(new yours_lidar_detect_human::YoursLidarDetectHuman(n));
#if 0
    std::map<std::string, int> tmap;
    for (int i = 0; i < 10; i++){
        tmap[std::to_string(i)] = i;
    }
    int cnt = 0;
    for (auto it : tmap)
    {
        tmap[it.first] = cnt * 2;
        //it.second = cnt * 2;
        cnt++;
    }
    for(auto it: tmap){
        ROS_INFO_STREAM( " 1 : " << it.first << " 2: " << it.second );
    }
#endif
    ros::Rate rate(10);
    while (ros::ok())
    {
        lidar->run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
