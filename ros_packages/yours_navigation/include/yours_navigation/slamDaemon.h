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

#ifndef SLAMDAEMON_H
#define SLAMDAEMON_H
#include <ros/ros.h>
#include <std_msgs/Int32.h>
class SlamDaemon
{
    enum SlamDaemonState{
        waitSlam = 0,
        loop,
        time_out
    };
    
    ros::NodeHandle mN;
    ros::Publisher testPub;
    ros::Subscriber slamStateSub;
    void slamStateCallBack(const std_msgs::Int32& msg);
    
    SlamDaemonState state;
    int timeOutCnt;
    
public:
    SlamDaemon(ros::NodeHandle n);
    ~SlamDaemon();
   /*
    static SlamDaemon& getInstance(ros::NodeHandle n){
        static SlamDaemon slamDaemon(n);
        return slamDaemon;
    }
    */
    void run();
};

#endif // SLAMDAEMON_H
