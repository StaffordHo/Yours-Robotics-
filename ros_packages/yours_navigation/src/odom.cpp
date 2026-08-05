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

#include "../include/yours_navigation/odom.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

bool isGetOdom;
nav_msgs::Odometry mOdomPose;
nav_msgs::Odometry mORBPose;
nav_msgs::Odometry mAmclPose;
double mOdomPoseYaw;
double mORBPosePitch;
double mAmclPoseYaw;
static bool isHaveAvoid = false;
static bool is_left_us = false;
static bool is_right_us = false;


std::mutex mOdomPoseMutex;
std::mutex mOdomYamMutex;

odom::odom(ros::NodeHandle& vi_n):
	mIsAmcl(true)
{
    mN = vi_n;
    odomSub = mN.subscribe("/odom", 1, &odom::OdomPoseCallBack, this);
    orbPoseSub = mN.subscribe("/yours_camera", 1, &odom::ORBPoseCallBack, this);
    //ultrasonicSub = mN.subscribe("/arm_ultrasonic_status", 1, &odom::UltrasonicCallBack, this);
    //kfUSPub = mN.advertise<riki_msgs::Ultrasonic>("/kfUS", 1);
    mForLogOdomPub = mN.advertise<std_msgs::Float64>("/yours_log_move_distance", 1);
    mGetPoseSub = mN.subscribe("/yours_get_current_pose", 1, &odom::GetPoseCallBack, this);
    mGetPosePub = mN.advertise<std_msgs::Float64MultiArray>("/yours_current_pose_for_init", 10);
    mPubPoseForLog = mN.advertise<std_msgs::Float64MultiArray>("/yours_log_pose",10);
    mPubPoseForZhiheng = mN.advertise<std_msgs::Float64MultiArray>("/zhiheng_robot_pose",10);
    usLeftSub = mN.subscribe("/yours_base/left_range", 1,
                             &odom::UltrasonicLeftCallBack, this);
    usRightSub = mN.subscribe("/yours_base/right_range", 1,
                             &odom::UltrasonicRightCallBack, this);
    std::string modeStr;
    mN.param("locate_mode", modeStr, std::string("amcl"));
	if(modeStr.compare("amcl")!=0){
		mIsAmcl = false;	
	}

    mN.param("ultrasonic_1_detect_threshold", mUltrasonic1Threshold, double(80.0));
    mN.param("ultrasonic_2_detect_threshold", mUltrasonic2Threshold, double(80.0));
    mN.param("ultrasonic_3_detect_threshold", mUltrasonic3Threshold, double(80.0));


    isGetOdom = false;

    kf1 = std::make_shared<cv::KalmanFilter>(2, 1, 0);
    kf1->transitionMatrix =  cv::Mat_<float>(2, 2) << 1, 1, 0, 1; //*cv::Mat(2,2,CV_32F, tMat);
    cv::setIdentity(kf1->measurementMatrix);                                             //测量矩阵H
    cv::setIdentity(kf1->processNoiseCov, cv::Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    cv::setIdentity(kf1->measurementNoiseCov, cv::Scalar::all(5e-4));                        //测量噪声方差矩阵R
    cv::setIdentity(kf1->errorCovPost, cv::Scalar::all(1));

    kf2 = std::make_shared<cv::KalmanFilter>(2, 1, 0);
    kf2->transitionMatrix =  cv::Mat_<float>(2, 2) << 1, 1, 0, 1; //*cv::Mat(2,2,CV_32F, tMat);
    cv::setIdentity(kf2->measurementMatrix);                                             //测量矩阵H
    cv::setIdentity(kf2->processNoiseCov, cv::Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    cv::setIdentity(kf2->measurementNoiseCov, cv::Scalar::all(5e-4));                        //测量噪声方差矩阵R
    cv::setIdentity(kf2->errorCovPost, cv::Scalar::all(1));

    kf3 = std::make_shared<cv::KalmanFilter>(2, 1, 0);
    kf3->transitionMatrix =  cv::Mat_<float>(2, 2) << 1, 1, 0, 1; //*cv::Mat(2,2,CV_32F, tMat);
    cv::setIdentity(kf3->measurementMatrix);                                             //测量矩阵H
    cv::setIdentity(kf3->processNoiseCov, cv::Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    cv::setIdentity(kf3->measurementNoiseCov, cv::Scalar::all(5e-4));                        //测量噪声方差矩阵R
    cv::setIdentity(kf3->errorCovPost, cv::Scalar::all(1));
    mTfListener = new tf::TransformListener();
}

odom::~odom()
{
    delete mTfListener;
}

double getDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    float deltX = 0;
    float deltY = 0;
    deltX = p2.x - p1.x;
    deltY = p2.y - p1.y;
    return sqrt((deltX * deltX) + (deltY * deltY));
}

void odom::OdomPoseCallBack(const nav_msgs::Odometry& msgs){
    //ROS_INFO("get odom");
    static int odomCnt = 0; 
    tf::Quaternion q(msgs.pose.pose.orientation.x,msgs.pose.pose.orientation.y, msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w) ;
    double roll, pitch;
    //std::unique_lock<std::mutex> lockYam(mOdomYamMutex);
    //lockYam.try_lock();
    //{
    tf::Matrix3x3(q).getRPY(roll, pitch, mOdomPoseYaw);
    //}
    //lockYam.unlock();
    //std::unique_lock<std::mutex> lockPose(mOdomPoseMutex);
    //lockPose.try_lock();
    //{
    mOdomPose = msgs;
    //}
    //lockPose.unlock();
    static geometry_msgs::Point lastPointLog;
    if(mIsAmcl){
        tf::StampedTransform transform;
        try {
            //得到坐标odom和坐标base_link之间的关系
            //listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
            mTfListener->lookupTransform("map", "base_link",
                   ros::Time(0), transform);
            //mTfListener->lookupTransform("map", "base_link",
            //    ros::Time::now(), transform);
            
            isGetOdom = true;

            float tx=transform.getOrigin().x();
            float ty=transform.getOrigin().y();
            float tw=tf::getYaw(transform.getRotation());
            //ROS_INFO("get Robot x:%f, y:%f, angle:%f", tx, ty, tw); 

            geometry_msgs::Point p;
            p.x = tx;
            p.y = ty;
            p.z = 0.0;
            static geometry_msgs::Point lastPoint = p;

            if((odomCnt%10) == 0){
                //ROS_INFO_STREAM(ros::Time::now());
                double dis = getDistance(p, lastPoint);
                if(dis > 5.0)
                    dis = 0.0;
                std_msgs::Float64 dMsg;
                dMsg.data = dis;
                mForLogOdomPub.publish(dMsg); 
                lastPoint = p;

                dis = getDistance(p, lastPointLog);
                if (dis > 1.0)
                {
                    std_msgs::Float64MultiArray posePub;
                    posePub.data.push_back(tw);
                    posePub.data.push_back(tx);
                    posePub.data.push_back(ty);
                    mPubPoseForLog.publish(posePub);
                	lastPointLog = p;
                }
            }
            odomCnt++;

            geometry_msgs::Quaternion q;
            q.x = transform.getRotation().x();
            q.y = transform.getRotation().y();
            q.z = transform.getRotation().z();
            q.w = transform.getRotation().w();

            mAmclPose.pose.pose.position = p;
            mAmclPose.pose.pose.orientation = q;
            mAmclPoseYaw = tw;
            std_msgs::Float64MultiArray zhihengPose;
            zhihengPose.data.push_back(tx);
            zhihengPose.data.push_back(ty);
            zhihengPose.data.push_back(tw);
            mPubPoseForZhiheng.publish(zhihengPose);

        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("OdomPoseCallBack error : %s",ex.what());
            ros::Duration(3.0).sleep();
        }
    }

}

nav_msgs::Odometry odom::getOdomPose(){
    //std::unique_lock<std::mutex> lockPose(mOdomPoseMutex);
    //lockPose.try_lock();
    return mOdomPose;
}

nav_msgs::Odometry odom::getORBPose(){
    return mORBPose;
}

double odom::getOdomYaw(){
    //std::unique_lock<std::mutex> lockYam(mOdomYamMutex);
    //lockYam.try_lock();
    return mOdomPoseYaw;
}


double odom::getSeslamPitch(){
    return mORBPosePitch;
}

bool odom::getPoseReady()
{
    return isGetOdom;
    
}


void odom::ORBPoseCallBack(const geometry_msgs::PoseStampedConstPtr& msgs){
    geometry_msgs::Pose pose =  msgs->pose;
    
    tf::Quaternion q(pose.orientation.x,pose.orientation.y,
                     pose.orientation.z,pose.orientation.w);
    double slam_yaw;
    double slam_pitch;
    double slam_roll;
    
    tf::Matrix3x3(q).getRPY(slam_roll, slam_pitch, slam_yaw);
    mORBPosePitch = slam_yaw;
    mORBPose.pose.pose = pose;
    mORBPose.header = msgs->header;
//    mORBPose.child_frame_id = "/odom";
    
    //ROS_INFO("roll = %lf, pitch = %lf, yaw = %lf", slam_roll * 180.0/ M_PI, slam_pitch * 180.0/ M_PI,  slam_yaw * 180.0/ M_PI);
    
}

#if 0
void odom::UltrasonicCallBack(const riki_msgs::UltrasonicConstPtr& msgs){
    //todo filter ultrasonic;
	static int avoidCnt = 0;
	static int noObstacleCnt = 0;
    cv::Mat measurement(1,1,CV_32F);

    cv::Mat preMat = kf1->predict();
    measurement.at<float>(0) = msgs->sonar1;
    kf1->correct(measurement);
    float us1 = preMat.at<float>(0);

    preMat = kf2->predict();
    measurement.at<float>(0) = msgs->sonar2;
    kf2->correct(measurement);
    float us2 = preMat.at<float>(0);

    preMat = kf3->predict();
    measurement.at<float>(0) = msgs->sonar3;
    kf3->correct(measurement);
    float us3 = preMat.at<float>(0);
/*
    ROS_INFO(" US1 %f  %f", msgs->sonar1, us1);
    ROS_INFO(" US2 %f  %f", msgs->sonar2, us2);
    ROS_INFO(" US3 %f  %f", msgs->sonar3, us3);
*/
    riki_msgs::Ultrasonic pubU;
    pubU.sonar1 = us1;
    pubU.sonar2 = us2;
    pubU.sonar3 = us3;
    kfUSPub.publish(pubU);

    //if(us1 < 60.0 || us2 < 60.0 || us3 < 60.0){
    //if(msgs->sonar1 < 80.0 || msgs->sonar2 < 80.0 || msgs->sonar3 < 80.0){
    //if(msgs->sonar1 < 50.0 || msgs->sonar2 < 50.0 || msgs->sonar3 < 50.0){

    if(msgs->sonar1 < mUltrasonic1Threshold ||  msgs->sonar2 < mUltrasonic2Threshold || msgs->sonar3 < mUltrasonic3Threshold){
    //if(msgs->sonar3 < 80.0 ){
        isHaveAvoid = true;
		if(avoidCnt == 10){
			noObstacleCnt = 1; //测试是这样延迟人感觉最好，没有停顿感，而且人走开后车辆走的效果也不错
		}else{
			noObstacleCnt = 0;
			avoidCnt++;
		}
    }else{
		if(noObstacleCnt < 1){
			avoidCnt = 0;
			isHaveAvoid = false;
		}
		else{
			noObstacleCnt--;
		}

	}
}
#endif

void odom::GetPoseCallBack(const std_msgs::Bool & msg){
    if(msg.data){
        ROS_INFO_STREAM("x : " << mAmclPose.pose.pose.position.x << " y: " << mAmclPose.pose.pose.position.y << " yaw: " << mAmclPoseYaw );
        std_msgs::Float64MultiArray msgPub;
        msgPub.data.push_back(mAmclPose.pose.pose.position.x);
        msgPub.data.push_back(mAmclPose.pose.pose.position.y);
        msgPub.data.push_back(mAmclPoseYaw);
        mGetPosePub.publish(msgPub); 
    }
}

bool odom::haveObstacles(){
	if((is_left_us == true) || (is_right_us == true)){
        return true;
    }else{
        return false;
    }
    //return false;
}

nav_msgs::Odometry odom::getAmclPose()
{
    return mAmclPose;
}

double odom::getAmclYaw()
{
    return mAmclPoseYaw;
}


void odom::UltrasonicLeftCallBack(const sensor_msgs::Range& msgs) {
    static unsigned int cnt = 0;
    if (msgs.range < 0.35) {
        cnt++;
    }else{
        cnt = 0;
    }
	if(cnt > 10){
        is_left_us = true;
    }else{
        is_left_us = false;
    }
}

void odom::UltrasonicRightCallBack(const sensor_msgs::Range& msgs) { 
    static unsigned int cnt = 0;
    if (msgs.range < 0.35) {
        cnt++;
    }else{
        cnt = 0;
    }
	if(cnt > 10){
        is_right_us = true;
    }else{
        is_right_us = false;
    }


}

bool odom::UltrasonicHaveObstacles(){
	if((is_left_us == true) || (is_right_us == true)){
        return true;
    }else{
        return false;
    }
}
