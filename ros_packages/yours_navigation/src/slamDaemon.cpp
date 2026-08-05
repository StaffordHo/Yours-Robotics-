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

#include "../include/yours_navigation/slamDaemon.h"

SlamDaemon::SlamDaemon(ros::NodeHandle n):
state(waitSlam)
,timeOutCnt(0)
{
    mN = n;
    slamStateSub = mN.subscribe("/yours_slam_state",1,&SlamDaemon::slamStateCallBack, this);
    ROS_INFO("init slam daemon");
}

SlamDaemon::~SlamDaemon()
{

}

void SlamDaemon::slamStateCallBack(const std_msgs::Int32& msg)
{
    timeOutCnt = 1;
    ROS_INFO("sub call %d", timeOutCnt);
    
}

void SlamDaemon::run()
{
    ROS_INFO(" %d %d ", state, timeOutCnt);
    switch (state){
        case waitSlam:{
            if(timeOutCnt == 1){
                state = loop;
            }
        }
        break;
        case loop:{
            timeOutCnt++;
            ROS_INFO("-----slam loop----- %d", timeOutCnt);
            if(timeOutCnt > 11){
                state = time_out;
            }
        }
        break;
        
        case time_out:{
            ROS_INFO("-----reboot slam-----");
            system("roslaunch ORB_SLAM2 yours_slam.launch");
            state = waitSlam;
        }
        break;
    }

}


