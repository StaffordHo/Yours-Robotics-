/*
 * Copyright 2019 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef ODOM_H
#define ODOM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
//#include <riki_msgs/Ultrasonic.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>

class odom
{
    void OdomPoseCallBack(const nav_msgs::Odometry& msgs);
    void ORBPoseCallBack(const geometry_msgs::PoseStampedConstPtr& msgs);
    //void UltrasonicCallBack(const riki_msgs::UltrasonicConstPtr& msgs);
    void GetPoseCallBack(const std_msgs::Bool & msg);
    ros::NodeHandle mN;
    ros::Subscriber odomSub;
    ros::Subscriber orbPoseSub;
    ros::Subscriber ultrasonicSub;
    ros::Subscriber mGetPoseSub;
    ros::Subscriber usLeftSub;
    ros::Subscriber usRightSub;

    ros::Publisher kfUSPub;
    ros::Publisher mForLogOdomPub;
    ros::Publisher mGetPosePub;
    ros::Publisher mPubPoseForLog;
    ros::Publisher mPubPoseForZhiheng;
    
    std::shared_ptr<cv::KalmanFilter>  kf1;
    std::shared_ptr<cv::KalmanFilter>  kf2;
    std::shared_ptr<cv::KalmanFilter>  kf3;

    double mUltrasonic1Threshold;
    double mUltrasonic2Threshold;
    double mUltrasonic3Threshold;

//    cv::Mat measurement = cv::Mat::zeros(1, 1, CV_32F);

    //std::mutex mOdomPoseMutex;
    //std::mutex mOdomYamMutex;
    
    tf::TransformListener* mTfListener;
    bool mIsGetPose;
	bool mIsAmcl;
    void UltrasonicLeftCallBack(const sensor_msgs::Range & msgs);
    void UltrasonicRightCallBack(const sensor_msgs::Range & msgs);
    
public:
    odom(ros::NodeHandle &vi_n);
    ~odom();
    static nav_msgs::Odometry getOdomPose();
    static double getOdomYaw();
    static bool getPoseReady();
    //static double getORBYaw();
    static double getSeslamPitch();
    static nav_msgs::Odometry getORBPose();
    static bool haveObstacles();
    static nav_msgs::Odometry getAmclPose();
    static double getAmclYaw();
    static bool UltrasonicHaveObstacles();
};

#endif // ODOM_H
