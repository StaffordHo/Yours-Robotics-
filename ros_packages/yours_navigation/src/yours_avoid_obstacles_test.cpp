#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include "../include/yours_navigation/followpath.h"

#define PI                  (3.1415926f)
#define ANGLE_PI            (180.0)

float getRadianFormAngle(float angle)
{
    return ((angle / ANGLE_PI) * PI);
}

nav_msgs::Odometry getNextAvoidObstaclePose(const nav_msgs::Odometry inPose_, double turnAngle_, double distance_){
    cv::Point2d basePoint;
    tf::Quaternion forwardQ(inPose_.pose.pose.orientation.x, inPose_.pose.pose.orientation.y, inPose_.pose.pose.orientation.z, inPose_.pose.pose.orientation.w);
    double forwardYaw = tf::getYaw(forwardQ);
    basePoint.x = cos( forwardYaw ) * distance_;
    basePoint.y = sin( forwardYaw ) * distance_;
    cv::Point2d goalPoint;
    
    goalPoint.x = basePoint.x + inPose_.pose.pose.position.x;
    goalPoint.y = basePoint.y + inPose_.pose.pose.position.y;

    double goalYaw =  forwardYaw + getRadianFormAngle(turnAngle_);  //yaw +  getRadianFormAngle(turnAngle_);
    tf::Quaternion goalQ = tf::createQuaternionFromYaw(goalYaw);

    nav_msgs::Odometry goalOdom;
    goalOdom.pose.pose.position.x = goalPoint.x;
    goalOdom.pose.pose.position.y = goalPoint.y;
    goalOdom.pose.pose.position.z = inPose_.pose.pose.position.z;
    goalOdom.pose.pose.orientation.x = goalQ.getX();
    goalOdom.pose.pose.orientation.y = goalQ.getY();
    goalOdom.pose.pose.orientation.z = goalQ.getZ();
    goalOdom.pose.pose.orientation.w = goalQ.getW();
    return goalOdom;
}

inline double getPoseYaw(const geometry_msgs::Pose& pose_){
    tf::Quaternion q(pose_.orientation.x, pose_.orientation.y,pose_.orientation.z, pose_.orientation.w);
    return tf::getYaw(q);
}

std::vector<AvoidPathCell> getAvoidObstaclePath(const nav_msgs::Odometry& currentPose, const bool& isTurnLeft, const double& turnAngle, const double& sideDistance,geometry_msgs::PoseArray& debugPoseArray){
    std::vector<AvoidPathCell> paths;
    debugPoseArray.poses.clear();
    debugPoseArray.header.frame_id = "map";

    nav_msgs::Odometry basePose;
    nav_msgs::Odometry goalPose1;
    nav_msgs::Odometry goalPose2;
    nav_msgs::Odometry goalPose3;

    tf::Quaternion currentQ(currentPose.pose.pose.orientation.x, currentPose.pose.pose.orientation.y, currentPose.pose.pose.orientation.z, currentPose.pose.pose.orientation.w );
    double currentYaw = tf::getYaw(currentQ);

    basePose.pose.pose.position = currentPose.pose.pose.position;

    tf::Quaternion baseYaw = tf::createQuaternionFromYaw(getRadianFormAngle(turnAngle) + currentYaw);//  (0,0,0,1);

    basePose.pose.pose.orientation.x = baseYaw.x();
    basePose.pose.pose.orientation.y = baseYaw.y();
    basePose.pose.pose.orientation.z = baseYaw.z();
    basePose.pose.pose.orientation.w = baseYaw.w();

    goalPose1 = getNextAvoidObstaclePose(basePose, -turnAngle, sideDistance);
    goalPose2 = getNextAvoidObstaclePose(goalPose1, -turnAngle, sideDistance);
    goalPose3 = getNextAvoidObstaclePose(goalPose2, turnAngle, sideDistance);

    geometry_msgs::Pose p0 = currentPose.pose.pose;
    geometry_msgs::Pose p1 = basePose.pose.pose;
    geometry_msgs::Pose p2 = goalPose1.pose.pose;
    geometry_msgs::Pose p3 = goalPose2.pose.pose;
    geometry_msgs::Pose p4 = goalPose3.pose.pose;

    geometry_msgs::PoseArray pa;
    pa.poses.push_back(p0);
    pa.poses.push_back(p1);
    pa.poses.push_back(p2);
    pa.poses.push_back(p3);
    pa.poses.push_back(p4);
    pa.header.frame_id = "map";
    debugPoseArray = pa;

    AvoidPathCell cell;
    cell.isTurn = true;
    cell.turnAngle = turnAngle;
    paths.push_back(cell);
    cell.isTurn = false;
    cell.goalPoint.point.x = goalPose1.pose.pose.position.x;
    cell.goalPoint.point.y = goalPose1.pose.pose.position.y;
    cell.goalPoint.point.yaw = getPoseYaw(p2);
    cell.goalPoint.speed = 0.4;
    cell.goalPoint.stopWidth = 0.6;
    cell.goalPoint.stopHeight = 1.0;
    paths.push_back(cell);
    cell.goalPoint.point.x = goalPose2.pose.pose.position.x;
    cell.goalPoint.point.y = goalPose2.pose.pose.position.y;
    cell.goalPoint.point.yaw = getPoseYaw(p3);
    paths.push_back(cell);
    cell.goalPoint.point.x = goalPose3.pose.pose.position.x;
    cell.goalPoint.point.y = goalPose3.pose.pose.position.y;
    cell.goalPoint.point.yaw = getPoseYaw(p4);
    paths.push_back(cell);
    return paths;
}

cv::Point getPointFormOdom(const nav_msgs::Odometry& odom){
    cv::Point p;
    p.x = odom.pose.pose.position.x;
    p.y = odom.pose.pose.position.y;
    return p;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "yours_ao_test");
    ros::NodeHandle n("~");
    ros::Rate rate(1);

    ros::Publisher pub;
    pub = n.advertise<geometry_msgs::PoseArray>("ao_path_test", 1);    

    cv::Mat image(400,400, CV_8UC3, cv::Scalar::all(0));
    nav_msgs::Odometry currentPose;
    double currentYaw = 20.0;
    tf::Quaternion currentQ = tf::createQuaternionFromYaw(getRadianFormAngle(currentYaw));
    currentPose.pose.pose.position.x = 2.0;
    currentPose.pose.pose.position.y = 2.0;
    currentPose.pose.pose.orientation.x = currentQ.x();
    currentPose.pose.pose.orientation.y = currentQ.y();
    currentPose.pose.pose.orientation.z = currentQ.z();
    currentPose.pose.pose.orientation.w = currentQ.w();



/*
    nav_msgs::Odometry basePose;
    nav_msgs::Odometry goalPose1;
    nav_msgs::Odometry goalPose2;
    nav_msgs::Odometry goalPose3;

    basePose.pose.pose.position = currentPose.pose.pose.position;

    tf::Quaternion baseYaw = tf::createQuaternionFromYaw(getRadianFormAngle(45.0 + currentYaw));//  (0,0,0,1);

    basePose.pose.pose.orientation.x = baseYaw.x();
    basePose.pose.pose.orientation.y = baseYaw.y();
    basePose.pose.pose.orientation.z = baseYaw.z();
    basePose.pose.pose.orientation.w = baseYaw.w();

    goalPose1 = getNextAvoidObstaclePose(basePose, -45.0, 2);
    goalPose2 = getNextAvoidObstaclePose(goalPose1, -45.0, 2);
    goalPose3 = getNextAvoidObstaclePose(goalPose2, 45.0, 2);

    geometry_msgs::Pose p0 = currentPose.pose.pose;
    geometry_msgs::Pose p1 = basePose.pose.pose;
    geometry_msgs::Pose p2 = goalPose1.pose.pose;
    geometry_msgs::Pose p3 = goalPose2.pose.pose;
    geometry_msgs::Pose p4 = goalPose3.pose.pose;

    geometry_msgs::PoseArray pa;
    pa.poses.push_back(p0);
    pa.poses.push_back(p1);
    pa.poses.push_back(p2);
    pa.poses.push_back(p3);
    pa.poses.push_back(p4);
    pa.header.frame_id = "map";
*/
    geometry_msgs::PoseArray pa;
    std::vector<AvoidPathCell> paths = getAvoidObstaclePath(currentPose, true, 60.0, 1.0,pa);
    double sideDistance = 1.0;
    double allDistance =  (2*sideDistance * cos(getRadianFormAngle(60.0)))   + sideDistance;
    std::cout << "avoid distance : " << allDistance << std::endl;   

    std::cout << paths.size() << std::endl;

    for(auto it:paths){
        it.debugPrint();
    }

    while(ros::ok()){
        cv::imshow("test", image);
        cv::waitKey(0);
        pub.publish(pa);
        ros::spinOnce();
        rate.sleep();
    }


return 0;
}
