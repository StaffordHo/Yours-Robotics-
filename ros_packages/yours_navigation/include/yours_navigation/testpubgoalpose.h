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

#ifndef TESTPUBGOALPOSE_H
#define TESTPUBGOALPOSE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include "macro.h"


enum MissionType{
    FromSellingToCharging = 0,
    FromChargingToSelling,
    Selling,
    FromSellingToReplenish,
    FromReplenishToSelling
};

class TestPubGoalPose
{
    ros::NodeHandle mN;
   
    ros::Publisher pub;
    ros::Subscriber subGoal;
    ros::Publisher testNextPosePub;
    
    void yoursGoalCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
    
//    geometry_msgs::PoseArray mPoseArray;
public:
    TestPubGoalPose(ros::NodeHandle& vi_n);
    ~TestPubGoalPose();
    void pubArray();
    geometry_msgs::PoseArray mPoseArray;
    std::map<int, geometry_msgs::PoseArray> mPoseArrayMap;
    int findNearestPoseID(const geometry_msgs::Pose& vi_pose);
    
    /**
     @param isGetGoal 使用者清除
     */
    bool isGetGoal;
    
    //test interface
    void pubTestNextPose(int pathCode, int poseCode);
};

#endif // TESTPUBGOALPOSE_H
