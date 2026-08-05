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

#include "../include/yours_navigation/yourslidardetect.h"
#include <std_msgs/UInt8.h>
#include <opencv2/opencv.hpp>

double distance(cv::Point2d p1, cv::Point2d p2){
     return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


YoursLidarDetect::YoursLidarDetect(ros::NodeHandle& n_):
n(n_)
,mFilterSlowCnt(0)
,mFilterStopCnt(0)
,mFilterPushCnt(0)
{
    mLaserSub = n.subscribe("/scan",1, &YoursLidarDetect::laserCallBack, this);
    mDetectDebugPub = n.advertise<sensor_msgs::LaserScan>("/debug_scan",1);
    mDetectSlowDownDebugPub= n.advertise<sensor_msgs::LaserScan>("/debug_scan_slow_down",1);
    mDetectPushRobotDebugPub= n.advertise<sensor_msgs::LaserScan>("/debug_scan_push_robot",1);
    mStopPub = n.advertise<std_msgs::UInt8>("/yours_laser_detect_stop_slow_down",10);
    mScanObstaclesInfoPub = n.advertise<std_msgs::Float32MultiArray>("/yours_laser_detect_info",10);

//    mSlowDownPub = n.advertise<std_msgs::Bool>("/yours_laser_detect_slow_down",10);
    
    n.param("stop_w",     m_stop_w,     0.5f);
    n.param("stop_h",     m_stop_h,     1.0f);
    n.param("stop_hp",    m_stop_hp,    0.0f);
    n.param("slow_down_w",m_slow_down_w,1.0f);
    n.param("slow_down_h",m_slow_down_h,1.6f);
    n.param("slow_down_hp",m_slow_down_hp,0.5f);
    n.param("debug_laser",mIsDebugLaser,true);
    
    n.param("stop_threshold", mStopThreshold, 60);
    n.param("slow_down_threshold", mSlowDownThreshold, 60);
    n.param("push_threshold", mPushThreshold, 60);
    
    n.param("filter_cnt", mFilterCntThreshold, 10);
}

YoursLidarDetect::~YoursLidarDetect()
{

}

void YoursLidarDetect::laserCallBack(const sensor_msgs::LaserScan& msgs)
{
    mLaser = msgs;
    mIsGetLaser = true;

}

void YoursLidarDetect::run(int& lidarDetectCode, std_msgs::Float32MultiArray& lidarDetectDataArray)
{
    int stopCnt = 0;
    int slowDownCnt = 0;
    std_msgs::UInt8 stopSlowMsg;
    //std_msgs::Bool slowMsg;

    stopSlowMsg.data = 0x00;

    std::pair<double, int> frontDistanceBuffer;
    double frontMaxLeft = 0;
    double frontMaxRight = 0;
    double sideLeftMinDistance = 999.;
    double sideRightMinDistance = -999.;
    double sideLeftMaxForward = 0;
    double sideRightMaxForward = 0;

    cv::Point2d lastLeftPoint(.0, .0);
    cv::Point2d lastRightPoint(.0, .0);
    bool get1stRight = false;
    //ROS_INFO("++");

    int pushRobotCnt = 0;

    if (mIsGetLaser)
    {
        mIsGetLaser = false;
        mSlowDownLaser = mLaser;
        mPushRobotLaser = mLaser;
        for (int i = 0; i < mLaser.ranges.size(); i++)
        {
            double angle = mLaser.angle_min + (i * mLaser.angle_increment);
            double x = cos(angle) * mLaser.ranges[i];
            double y = sin(angle) * mLaser.ranges[i];
            if (x < m_stop_h && x > -m_stop_hp && fabs(y) > 0.01 )
            {
                if ( (fabs(y) < (m_stop_w / 2.0) )  )
                {
                    stopCnt++;
                }
                else
                {
                    mLaser.ranges[i] = 0;
                }
                //检测前方平均距离和前方最左和最右, 检测左右第一个连续的激光线段
                if (x < m_stop_h && x > 0 && (fabs(y) < 2.0) )
                {
                    frontDistanceBuffer.first = frontDistanceBuffer.first + x;
                    frontDistanceBuffer.second++;
                    if (y > 0)
                    {
                        //前方最左
                        cv::Point2d currentLeft(x, y);
                        double distL = distance(currentLeft, lastLeftPoint);
                        if (lastLeftPoint.x > 0.0001 && lastLeftPoint.y > 0.0001)
                        {
                            if (distL < 0.6)
                            {
                                lastLeftPoint = currentLeft;
                                if (y > frontMaxLeft)
                                {
                                    frontMaxLeft = y;
                                }
                            }
                        }
                        else if (lastLeftPoint.x < 0.0001 && lastLeftPoint.y < 0.0001)
                        {
                            lastLeftPoint = currentLeft;
                            //ROS_INFO("??");
                        }
                    }
                    else
                    {
//前方最右
#if 0
                                if (y < frontMaxRight)
                                {
                                    frontMaxRight = y;
                                }
#endif
                        cv::Point2d currentRight(x, y);
                        double distR = distance(currentRight, lastRightPoint);
                        //if (lastRightPoint.x > 0.0001 && lastRightPoint.y < -0.0001)
                        //{
                        if(distR < 0.){
                        }
                        if (distR < 0.6)
                        {
                            lastRightPoint = currentRight;
                            if (y < frontMaxRight)
                            {
                                frontMaxRight = y;
                            }
                        }else{
                            //ROS_INFO_STREAM(frontMaxRight << "  " << y );

                        }
                        //}
                        //else
                        if (get1stRight == false)
                        {
                            get1stRight = true;
                            lastRightPoint = currentRight;
                            //ROS_INFO("!!");
                        }
                        
                    }
                }

                if (fabs(y) < (m_stop_w / 2.0))
                {
                    if (y > 0)
                    {
                        //检测车左面
                        if (y < sideLeftMinDistance)
                        {
                            sideLeftMinDistance = y;
                        }
                        if (x > sideLeftMaxForward)
                        {
                            sideLeftMaxForward = x;
                        }
                    }
                    else
                    {
                        //检测车右面
                        if (y > sideRightMinDistance)
                        {
                            sideRightMinDistance = y;
                        }
                        if (x > sideRightMaxForward)
                        {
                            sideRightMaxForward = x;
                        }
                    }
                }
            }
            else
            {
                mLaser.ranges[i] = 0;
            }
            if (x > -m_slow_down_hp && x < m_slow_down_h && fabs(y) < (m_slow_down_w / 2.0))
            {
                mSlowDownLaser.ranges[i] = mSlowDownLaser.ranges[i];
                slowDownCnt++;
            }
            else
            {
                mSlowDownLaser.ranges[i] = 0;
            }

            if (x < 0.3 && x > -0.1 && fabs(y) > 0.01)
            {
                if (fabs(y) < 0.25)
                {
                    pushRobotCnt++;
                }
                else
                {
                    mPushRobotLaser.ranges[i] = 0;
                }
            }
            else
            {
                mPushRobotLaser.ranges[i] = 0;
            }
        }
        
        if(pushRobotCnt > mPushThreshold){
            mFilterPushCnt++;
        }else{
            mFilterPushCnt = 0;
        }

        if(mFilterPushCnt > mFilterCntThreshold){
            stopSlowMsg.data = stopSlowMsg.data | 0x04;
            mFilterPushCnt = mFilterCntThreshold + 2;
        }

        //ROS_INFO("stop cnt %d, slow down cnt %d", stopCnt, slowDownCnt );
        if (stopCnt > mStopThreshold)
        {
            mFilterStopCnt++; 
        //    stopSlowMsg.data = stopSlowMsg.data | 0x02;
        }else{
            mFilterStopCnt = 0;
        }

        if (mFilterStopCnt > mFilterCntThreshold)
        {
            stopSlowMsg.data = stopSlowMsg.data | 0x02;
            mFilterStopCnt = mFilterCntThreshold + 2;
        }

        if (slowDownCnt > mSlowDownThreshold)
        {
            //stopSlowMsg.data = stopSlowMsg.data | 0x01;
            mFilterSlowCnt++;
        }else{
            mFilterSlowCnt = 0;
        }

        if(mFilterSlowCnt > mFilterCntThreshold){
            stopSlowMsg.data = stopSlowMsg.data | 0x01;
            mFilterSlowCnt = mFilterCntThreshold + 2;
        }


        mLaser.header.stamp = ros::Time::now();
        if (mIsDebugLaser)
        {
            mDetectDebugPub.publish(mLaser);
            mDetectSlowDownDebugPub.publish(mSlowDownLaser);
            mDetectPushRobotDebugPub.publish(mPushRobotLaser);
        }
    }
    else
    {
        stopSlowMsg.data = 0x03;
    }

    lidarDetectCode = stopSlowMsg.data;
    mStopPub.publish(stopSlowMsg);

    double frontDistance = 0.0;
    if (frontDistanceBuffer.second != 0)
    {
        frontDistance = frontDistanceBuffer.first / (double)frontDistanceBuffer.second;
    }

    std_msgs::Float32MultiArray dataArray;
    dataArray.data.push_back(frontDistance);
    dataArray.data.push_back(frontMaxLeft);
    dataArray.data.push_back(frontMaxRight);
    dataArray.data.push_back(sideLeftMinDistance);
    dataArray.data.push_back(sideRightMinDistance);
    dataArray.data.push_back(sideLeftMaxForward);
    dataArray.data.push_back(sideRightMaxForward);
    mScanObstaclesInfoPub.publish(dataArray);
    lidarDetectDataArray = dataArray;
    //if(pushRobotCnt != 0){
    //ROS_INFO_STREAM(" push  " << pushRobotCnt);
    //}
}

int YoursLidarDetect::setStopParam(float stopWight, float stopHeight){
     m_stop_w = stopWight;
     m_stop_h = stopHeight;

     return 0;
}
