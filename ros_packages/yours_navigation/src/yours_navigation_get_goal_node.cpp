#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

void goalCallBack(const geometry_msgs::PoseStamped& msg){
    ROS_INFO_STREAM(msg.pose);
}


int main(int argc, char** argv){
   
    ros::init(argc, argv, "yours_navigation_get_goal");
    ros::NodeHandle n;
    
    ros::Subscriber mRvizGoalSub = n.subscribe("/move_base_simple/goal", 1, &goalCallBack);
   
    ros::spin();
    
    return 0;
}
