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

#include "../include/yours_navigation/testpubgoalpose.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
//#include <yours_message/TestNextGoal.h>

//#define GOAL_FROM_FILE 1

TestPubGoalPose::TestPubGoalPose(ros::NodeHandle& vi_n):isGetGoal(false)
{
    mN = vi_n;
    pub = mN.advertise<geometry_msgs::PoseArray>("/yours_goal", 1);
//     testNextPosePub = mN.advertise<yours_message::TestNextGoal>("/yours_test_next_goal", 1);
    geometry_msgs::Point p;
    geometry_msgs::Quaternion q;
    
   // mN.param("goal_path", )
    
#if SIM_ORB_GOAL == 1
#ifndef GOAL_FROM_FILE
    subGoal = mN.subscribe("/yours_goal_manual_set", 5, &TestPubGoalPose::yoursGoalCallBack, this);
#endif
#else
    p.x = 5.;
    p.y = 0.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 0.707;
    q.w = 0.707;
    
    geometry_msgs::Pose p1;
    p1.position = p;
    p1.orientation = q;
    
    p.x = 5.;
    p.y = 5.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 1.;
    q.w = 0.;
    
    geometry_msgs::Pose p2;
    p2.position = p;
    p2.orientation = q;
    
    p.x = 0.;
    p.y = 5.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 0.707;
    q.w = -0.707;
    
    geometry_msgs::Pose p3;
    p3.position = p;
    p3.orientation = q;

    p.x = 0.;
    p.y = 0.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 0.;
    q.w = 1.;

    geometry_msgs::Pose p4;
    p4.position = p;
    p4.orientation = q;
   
    mPoseArray.poses.push_back(p1);
    mPoseArray.poses.push_back(p2);
    mPoseArray.poses.push_back(p3);
    mPoseArray.poses.push_back(p4);
    
    p.x = 5.;
    p.y = 0.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 0.707;
    q.w = -0.707;
    
    geometry_msgs::Pose p5;
    p5.position = p;
    p5.orientation = q;
    
    p.x = 5.;
    p.y = -5.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 1.;
    q.w = 0.;
    
    geometry_msgs::Pose p6;
    p6.position = p;
    p6.orientation = q;
    
    p.x = 0.;
    p.y = -5.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 0.707;
    q.w = 0.707;
    
    geometry_msgs::Pose p7;
    p7.position = p;
    p7.orientation = q;
   
    p.x = 0.;
    p.y = 0.;
    p.z = 0.;
    q.x = 0.;
    q.y = 0.;
    q.z = 0.;
    q.w = 1.;
    
    geometry_msgs::Pose p8;
    p8.position = p;
    p8.orientation = q;
    
    mPoseArray.poses.push_back(p5);
    mPoseArray.poses.push_back(p6);
    mPoseArray.poses.push_back(p7);
    mPoseArray.poses.push_back(p8);
#endif
}

TestPubGoalPose::~TestPubGoalPose()
{

}

void TestPubGoalPose::pubArray()
{
    static unsigned int seq;
    
    for(auto it:mPoseArrayMap){
        geometry_msgs::PoseArray willPub;
        
        willPub.header.frame_id = "/odom";
        willPub.header.stamp = ros::Time::now();
        willPub.header.seq = seq++;
        willPub.poses = it.second.poses;
        if(willPub.poses.size()!=0)
        pub.publish(willPub);
    }
    //mPoseArray.header.frame_id = "/odom";
    //mPoseArray.header.stamp = ros::Time::now();
    //mPoseArray.header.seq = seq++;
    //pub.publish(mPoseArray);
}


int TestPubGoalPose::findNearestPoseID(const geometry_msgs::Pose& vi_pose)
{
    int tempID;
    double minDis = 999.0;

    return -1;
}

void TestPubGoalPose::yoursGoalCallBack(const geometry_msgs::PoseArrayConstPtr& msg){
    mPoseArray.poses.clear();
    for(int i = 0; i < msg->poses.size(); i++){
        mPoseArray.poses.push_back(msg->poses[i]);
    }
  
    std::string c_string = msg->header.frame_id;
    //std::string = std::string( id.data.c_str() );
    if(c_string.compare("0") == 0){
        mPoseArrayMap.clear();
    }
    int id_int = std::atoi(c_string.c_str());
    mPoseArrayMap[id_int] = mPoseArray;
    if(c_string.compare("4") ==  0){
        for(auto it:mPoseArrayMap){
            std::cout << it.first << std::endl;
            geometry_msgs::PoseArray data = it.second;
            std::cout << data.poses.size() << std::endl;
            /*
			for(int i = 0; i < data.poses.size() ;i++){
                ROS_INFO_STREAM ("data " << data.poses[i] );
            } 
			*/
            
            //for(auto itsub:it. ){
            //    ROS_INFO_STREAM ("data " << itsub );
                
            //}
        }
        isGetGoal = true;
        this->pubArray();
    }
    
}

void TestPubGoalPose::pubTestNextPose(int pathCode, int poseCode)
{
    //yours_message::TestNextGoal nextGoal;
    //nextGoal.path_code.data =pathCode;
    //nextGoal.pose_code.data = poseCode;
    //testNextPosePub.publish(nextGoal);

}
