/*
 * Copyright 2020 <copyright holder> <email>
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

#ifndef YOURSLIDARDETECT_H
#define YOURSLIDARDETECT_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>

class YoursLidarDetect
{
    ros::NodeHandle n;
    ros::Subscriber mLaserSub;
    ros::Publisher mDetectDebugPub;
    ros::Publisher mDetectSlowDownDebugPub;
    ros::Publisher mDetectPushRobotDebugPub;
    ros::Publisher mStopPub;
    ros::Publisher mScanObstaclesInfoPub; //扫描到的障碍物物体信息发布，用于避障模块
//    ros::Publisher mSlowDownPub;
    
    void laserCallBack(const sensor_msgs::LaserScan& msgs);
    sensor_msgs::LaserScan mLaser;
    sensor_msgs::LaserScan mSlowDownLaser;
    sensor_msgs::LaserScan mPushRobotLaser;
    bool mIsGetLaser;
    
    //停止区域宽，高 hp是雷达向后的距离
    float m_stop_w;
    float m_stop_h;
    float m_stop_hp;
    
    //减速区域宽，高
    float m_slow_down_w;
    float m_slow_down_h;
    float m_slow_down_hp;
    
    bool mIsDebugLaser;
    
    int mStopThreshold;
    int mSlowDownThreshold;
    int mPushThreshold;
    int mFilterStopCnt;
    int mFilterSlowCnt;
    int mFilterPushCnt;
    int mFilterCntThreshold;

public:
    YoursLidarDetect(ros::NodeHandle& n_);
    ~YoursLidarDetect();
    void run(int& lidarDetectCode, std_msgs::Float32MultiArray& lidarDetectDataArray);
    int setStopParam(float stopWight, float stopHeight);
    
};

#endif // YOURSLIDARDETECT_H
