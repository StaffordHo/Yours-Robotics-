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

#include "../include/yours_navigation/plan.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
//#include <yours_message/Goal.h>
#include <eigen3/Eigen/Eigen>
//#include <binorobot_msgs/YoursNavPoint.h>
//#include <binorobot_msgs/YoursNavPointArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Float64.h>
//#define  USERSCAN

Plan::Plan(ros::NodeHandle& vi_n, const bool& is_remap): localPlanState(idle) 
,mSlamState(TRACKING_OK)
,isNavStatus(false)
,userScan(false)
,mLoseProcessing(std::make_shared<LoseProcessing>())
,isFirstTracking(true)
,isCanUpdateYaw(true)
,mIsToCharging(false)
,mIsToReplenish(false)
,mIsTestTurn(false)
,mIsTestRun(false)
,mTestTurnData(0)
,mRadianBuffer(0.0) 
,mIsEnableLose(true)
,mIsEnableHaveObstaclesStopRobot(true)
,mIsEnableHaveObstaclesStopRobotInt(1)
,isFirstGetSlamState(true)
,mMaxObstaclePixel(0)
,mFarthestObstacle(0)
,mSlamHeartbeat(-1)
,mIsAmclLocation(false)
,mIsGetRvizGoal(false)
,mIsStop(false)
,mIsSlowDown(false)
,isHaveAmclPath(false)
,mIsEnableLidarStop(true)
,mIsToReDock(false)
,isGetDangerZone(false)
,mIsInitPath(false)
,mIsStartWork(false)
,mAMCLRunStatus(-1)
,mArmErrorDetect(std::make_shared<yours_detect_arm_error_code::YoursDetectArmErrorCode>(vi_n))
,mLog(std::make_shared<yours_log::YoursLog>(std::string(getenv("HOME")) + "/log/yours_plan/"))
,mFollowPathState(FPState::InitPath)
{
    mN = vi_n;
    //mTestPub = std::make_shared<TestPubGoalPose>(vi_n);
    //ros::Subscriber subOdom = 
    //odomSub = vi_n.subscribe("/odom", 1, &Plan::OdomPoseCallBack, this);
    tp = mN.advertise<std_msgs::String>("/pub_test", 1);
    //mPubRgbImage = mN.advertise<sensor_msgs::Image>("/yours_rgb_image", 1);
    mPubMissionRecall = mN.advertise<std_msgs::Int32>("/yours_mission_recall", 1);
    
	//rgbSub = mN.subscribe("/camera/color/image_raw/compressed", 1,&Plan::RgbImageCallBack, this);
    //rgbSub = mN.subscribe("/camera/infra1/image_rect_raw/compressed", 1,&Plan::RgbImageCallBack, this);
    goalSub = mN.subscribe("/goal_call", 1,&Plan::GoalCallBack, this);
    scanCodeSub = mN.subscribe("/scan_code", 1,&Plan::ScanCodeCallBack, this);
    robotPoseSub = mN.subscribe("/vo",1,&Plan::RobotPoseCallBack, this);
    startWorkSub = mN.subscribe("/start_work", 1, &Plan::StartWorkCallBack, this);
#if SIM_DEBUG == 1
    twistPub = mN.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
#else
  twistPub = mN.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  //twistPub = mN.advertise<geometry_msgs::Twist>("/navigation_cmd_vel", 1000);
#endif
    slamStateSub = mN.subscribe("/yours_slam_state", 1, &Plan::SlamStateCallBack, this);
    toChargeSub = mN.subscribe("/yours_to_charge", 1, &Plan::ToChargeCallBack, this);
    toReplenishSub = mN.subscribe("/yours_to_replenish", 1, &Plan::ToReplenishCallBack, this);
   
    testTurnSub = mN.subscribe("/yours_test_turn", 1, &Plan::TestTurnCallBack, this);
    testRunSub = mN.subscribe("/yours_test_run", 1, &Plan::TestRunCallBack, this);
    mSubMission = mN.subscribe("/yours_mission_code", 1, &Plan::MissionCallBack, this);
    
    goalCallPub = mN.advertise<std_msgs::Bool>("/goal_call", 1);
    
    mVisionPixelPub = mN.advertise<std_msgs::Int32>("/yours_vision_obstacle", 1);
    mRvizGoalSub = mN.subscribe("/move_base_simple/goal", 1, &Plan::RvizGoalCallBack,this);
    
    mGoalArrayPub = mN.advertise<geometry_msgs::PoseArray>("/yours_goal_array",1);
    mLaserDetectSub = mN.subscribe("/yours_laser_detect_stop_slow_down",1,&Plan::LaserDetectCallBack, this);
    
    //mYoursNavPointSub = mN.subscribe("/yours_nav_points",1,&Plan::YoursNavPointCallBack, this);
    mYoursNavPointCheckPub = mN.advertise<std_msgs::Int32>("/yours_nav_point_check", 1);
    //mSoundCodeSub = mN.subscribe("/yours_sound_code", 1, &Plan::SoundCodeCallback, this);

    mZhihengPoseArraySub = mN.subscribe("/zhiheng_goal_array", 1, &Plan::ZhihengPoseCallBack, this) ;
    mZhihengStartWorkSub = mN.subscribe("/zhiheng_robot_ctrl",1, &Plan::ZhihengStartWorkCallBack, this);
    mZhihengStatePub = mN.advertise<std_msgs::UInt8MultiArray>("/zhiheng_robot_ctrl_recall", 1);

    mMoveSub = mN.subscribe("/move_base/result", 1, &Plan::MoveBaseCallBack, this);
    mMoveBaseVelSub = mN.subscribe("/move_base_cmd_vel", 1, &Plan::MoveBaseVelCallBack, this);
    mNavNegativePub = mN.advertise<std_msgs::Float64>("/yours_nav_negative", 1, true);

    mMoveBaseGoalPub = mN.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",1);
    mMoveBaseGoalMsg.header.frame_id = "map";
    mMoveBaseGoalMsg.goal.target_pose.header.frame_id = "map";


    //Dangerzone display
    _yoursDangerZone1 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_1", 1);
    _yoursDangerZone2 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_2", 1);
    _yoursDangerZone3 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_3", 1);
    _yoursDangerZone4 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_4", 1);
    _yoursDangerZone5 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_5", 1);

    _yoursDangerZone6 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_6", 1);
    _yoursDangerZone7 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_7", 1);
    _yoursDangerZone8 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_8", 1);
    _yoursDangerZone9 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_9", 1);
    _yoursDangerZone10 = mN.advertise<geometry_msgs::PolygonStamped>("/yours_danger_zone_10", 1);


    _yoursPubVector.push_back(_yoursDangerZone1);
    _yoursPubVector.push_back(_yoursDangerZone2);
    _yoursPubVector.push_back(_yoursDangerZone3);
    _yoursPubVector.push_back(_yoursDangerZone4);
    _yoursPubVector.push_back(_yoursDangerZone5);
    _yoursPubVector.push_back(_yoursDangerZone6);
    _yoursPubVector.push_back(_yoursDangerZone7);
    _yoursPubVector.push_back(_yoursDangerZone8);
    _yoursPubVector.push_back(_yoursDangerZone9);
    _yoursPubVector.push_back(_yoursDangerZone10);

    _yoursDangerZoneTest = mN.advertise<geometry_msgs::PoseArray>("/danger_zone_test",1);

    mUpdatePathSub = mN.subscribe("/yours_update_path", 1, &Plan::UpdatePath, this);

    mTestCmdSub = mN.subscribe("/yours_test_cmd",1,&Plan::TestCMDCallBack, this);

    mFollower = std::make_shared<followpath>(mN, mLog);
    mOdom = std::make_shared<odom>(mN);
    mVisionDetect = std::make_shared<yours_vision_detect_node>(mN);
    //mLidarDetect = std::make_shared<YoursLidarDetect>(mN);
    mLidarDetect = std::make_shared<yours_lidar_detect_human::YoursLidarDetectHuman>(mN);
   
    geometry_msgs::Pose gp;
    gp.position.z = 0.0;
    gp.orientation.x = 0.0;
    gp.orientation.y = 0.0;
    int goalNumber = 0;

    mN.param("goal_number", goalNumber, 0);
    for(int i = 0; i < goalNumber; i++){
        float x,y,z,w;
        mN.param("x_"+std::to_string(i), x, 0.0f);
        mN.param("y_"+std::to_string(i), y, 0.0f);
        mN.param("z_"+std::to_string(i), z, 0.0f);
        mN.param("w_"+std::to_string(i), w, 0.0f);
        
        gp.position.x = x;
        gp.position.y = y;
        gp.orientation.z = z;
        gp.orientation.w = w;
        mGoalPoseArray.poses.push_back(gp);
    }
    mGoalPoseArray.header.frame_id = "map";
    mGoalPoseArray.header.stamp = ros::Time::now();

    
    int MaxObstaclePixel = 2000;
    int FarthestObstacle = 800;
    
    mN.param("max_obstacle_pixel", mMaxObstaclePixel, MaxObstaclePixel);
    mN.param("farthest_obstacle", mFarthestObstacle, FarthestObstacle);
    mN.param("amcl_location", mIsAmclLocation, false);
    //std::string navPointPath;
    mN.param("amcl_nav_path", mNavPointPath, std::string("./"));
    mN.param("remap_path", mRemapPathFilePath, std::string("./"));
    
    mN.param("lidar_detect_enable", mIsEnableLidarStop, true);

    mN.param("have_auto_charge", isHaveAutoCharge, false);
    mN.param("have_buhuo_auto_charge", isHaveBuhuoAutoCharge, false);

    mLog->set_log_level(yours_log::debug);
    mLog->set_print_level(yours_log::error);

    if (!is_remap) {
        if (readLidarNavPoint(mNavPointPath, mPointMap, mDangerZone)) {
            fillRandomMissionArray(mPointMap, mRandomMissionArray);
            mFollower->updatePath(mPointMap, mRandomMissionArray);
            for (auto it = mPointMap.begin(); it != mPointMap.end(); it++) {
                for (int i = 0; i < mPointMap.at(it->first).size(); i++) {
                    YoursNavPoint p = mPointMap.at(it->first)[i];
                    geometry_msgs::Pose pose;
                    pose.position.x = p.point.x;
                    pose.position.y = p.point.y;
                    tf::Quaternion q = tf::createQuaternionFromYaw(p.point.yaw);
                    pose.orientation.w = q.w();
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    mGoalPoseArray.poses.push_back(pose);
                    ROS_INFO_STREAM(pose);
                }
            }

            this->drawZone(mDangerZone, _yoursPubVector);
            mFollower->updateDangerzone(mDangerZone);

            isHaveAmclPath = true;
            mFollower->zhihengUpdatePath(mGoalPoseArray);
            mLog->info("Navigation Path read ok");
        } else {
            ROS_INFO("amcl path read fail");
            mLog->info("Navigation Path read fail");
        }
    }

    if (is_remap) {
        mNavPointPath = mRemapPathFilePath;
        if (readLidarNavPoint(mRemapPathFilePath, mPointMap, mDangerZone)) {
            mGoalPoseArray.poses.clear();
            fillRandomMissionArray(mPointMap, mRandomMissionArray);
            mFollower->updatePath(mPointMap, mRandomMissionArray);
            for (auto it = mPointMap.begin(); it != mPointMap.end(); it++) {
                for (int i = 0; i < mPointMap.at(it->first).size(); i++) {
                    YoursNavPoint p = mPointMap.at(it->first)[i];
                    geometry_msgs::Pose pose;
                    pose.position.x = p.point.x;
                    pose.position.y = p.point.y;
                    tf::Quaternion q = tf::createQuaternionFromYaw(p.point.yaw);
                    pose.orientation.w = q.w();
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    mGoalPoseArray.poses.push_back(pose);
                    ROS_INFO_STREAM(pose);
                }
            }
        	isHaveAmclPath = true;
            mLog->info("Remap Path read ok");
        } else {
            isHaveAmclPath = false;
            mLog->info("Remap Path read fail");
        }
    }

    mImpactSensorSub = mN.subscribe("/yours_base/anti_collision", 1, &Plan::ImpactSensorCallback, this);
    
    robot_distance_ctrl_ = std::make_shared<YoursGetGoalRobotDistance>(mN, mPointMap);

    ROS_INFO("!! max_obstacle_pixel !! %d", mMaxObstaclePixel);
    ROS_INFO("!! farthest_obstacle !! %d", mFarthestObstacle);
    
    ROS_INFO("init plan");
    mLog->info("yours plan init ok");
}

Plan::~Plan(){
    
}
//call back start



//call back end

void Plan::GoalCallBack(const std_msgs::Bool& msg){
    isNavStatus = msg.data;
    if(isNavStatus )
        ROS_INFO(" Plan Is Nav ");
    else
        ROS_INFO(" Plan No Nav ");
    
}

void Plan::ScanCodeCallBack(const std_msgs::Bool& msg){
	userScan = msg.data;
}


void Plan::RgbImageCallBack(const sensor_msgs::CompressedImageConstPtr& msg){
    //ROS_INFO("get image");
    static unsigned int cnt = 0;
	cv::Mat rgb;
    try{
        rgb = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat small;
    cv::resize(rgb, small, cv::Size(320, 180));
    //cv::cvtColor(small, small, cv::COLOR_BGR2RGB);
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
    out_msg.image    = small; // Your cv::Mat

    if((cnt++)%5 == 0){
	    mPubRgbImage.publish(out_msg.toImageMsg());
    }
}

void Plan::RobotPoseCallBack(const nav_msgs::OdometryConstPtr& msg){
    geometry_msgs::Pose robotPose_ = msg->pose.pose;
    geometry_msgs::Quaternion q_ =  robotPose_.orientation;
    Eigen::Quaterniond  eigenQ(q_.w, q_.x, q_.y, q_.z);
    Eigen::Matrix3d rMat = eigenQ.toRotationMatrix();
    Eigen::Vector3d rVector = rMat.eulerAngles(0,1,2);
    //ROS_INFO(" %f, %f, %f", rVector.x(), rVector.y(), rVector.z()); //y是
    mRobotYaw = rVector.y(); //经测试，y轴是 yaw轴
}


void Plan::StartWorkCallBack(const std_msgs::Bool& msg)
{
    mIsStartWork = msg.data;
}

void Plan::ToChargeCallBack(const std_msgs::Bool& msg){
    mIsToCharging = msg.data;
}

void Plan::ToReplenishCallBack(const std_msgs::Bool& msg){
    mIsToReplenish = msg.data;
}

void Plan::MissionCallBack(const std_msgs::Int32& msg){
    std_msgs::Int32 pubData;
    pubData.data = 0;
    ROS_INFO_STREAM("get mission code : " << msg.data);
    if (msg.data == 401) {
        mIsStartWork = true;
        mIsToCharging = false;
        mIsToReplenish = false;
        mIsToReDock = false;
    } else if (msg.data == 402) {
        if(mIsToCharging){
            pubData.data = 4;
        }
        mIsStartWork = true;
        mIsToCharging = true;
        mIsToReplenish = false;
        mIsToReDock = false;
    } else if (msg.data == 403) {
        if(mIsToReplenish){
            pubData.data = 4;
        }
        mIsStartWork = true;
        mIsToReplenish = true;
        mIsToCharging = false;
        mIsToReDock = false;
    } else if (msg.data == 404) {
        mIsStartWork = false;
    } else if (msg.data == 410) {
        mIsToReDock = true;
        mIsStartWork = true; 
        mIsToCharging = false;
        mIsToReplenish = false;
        if (mAMCLRunStatus == 220 || mAMCLRunStatus == 221 || mAMCLRunStatus == 110 || mAMCLRunStatus == 111 || mAMCLRunStatus == 0)
        {
            pubData.data = 0;
        }
        else if(mAMCLRunStatus == 141){
            pubData.data = 4;
        }
        else
        {
            if(mFollowPathState == FPState::ReturnAutoDockStartPoint || mFollowPathState == FPState::AutoDock ){
                pubData.data = 4;
            }else{
            //小车不能执行此命令
                pubData.data = 7;
                mIsToReDock = false;
        		mIsStartWork = false; 
            }

        }
        if(!isHaveAutoCharge){
            pubData.data = 7;
            mIsToReDock = false;
        	mIsStartWork = false; 
        }   
    } else if (msg.data == 411) {
        mIsInitPath = true;
        mIsStartWork = true;
    } else {
        ROS_ERROR("mission call error  code: %d", msg.data);
    }
    if(msg.data != 404){
        if(mFollower->isHavePath()){
            if(mSlamState == TRACKING_OK){
                if(msg.data == 402){
                    if(mFollower->isCharging()){
                        pubData.data = 5;
                    }
                }else if(msg.data == 403){
                    if(mFollower->isReplenish()){
                        pubData.data = 5;
                    }
                }
            }else{
                pubData.data = 1;
            }
        }else{
           pubData.data = 2;
        }


    }else{
        pubData.data = 0;
    }
    mPubMissionRecall.publish(pubData);
    mLog->info("Get mission code : " + std::to_string(msg.data) + ". Retrun code : " + std::to_string(pubData.data));

}

//void Plan::TestTurnCallBack(const std_msgs::Bool& msg){
void Plan::TestTurnCallBack(const std_msgs::Int32& msg){
	mTestTurnData = msg.data;
	ROS_INFO("get data %d",mTestTurnData);
	if(mTestTurnData > 0)
		mIsTestTurn = true;// = msg.data;
	else
		mIsTestTurn = false;// = msg.data;



	if(mIsTestTurn){
        mRadianBuffer = odom::getOdomYaw();
        ROS_INFO("????%f", odom::getOdomYaw());
    }
}

void Plan::TestRunCallBack(const std_msgs::Int32& msg){
	mTestTurnData = msg.data;
	ROS_INFO("get data %d",mTestTurnData);
	if(mTestTurnData != 0)
		mIsTestRun = true;// = msg.data;
	else
		mIsTestRun = false;// = msg.data;

}


void Plan::Stop( geometry_msgs::Twist& inout_tw )
{
    inout_tw.angular.z = 0;
    inout_tw.linear.x = 0;
}

void Plan::stopStatus()
{
    /*
    if(odom::getSeslamStatus() == SLAM_COMMAND_RELOC_STOP)    //0
    {
        localPlanState = reloc;
        time_cnt = 0;
    } else{
        if(time_cnt == 0)
        {
            ROS_INFO("write stop");
            connent_tO_seslam->writerDataToSeslam(NAV_COMMAND_STOP,0);
        }
        wait();
    }
    */
}

void Plan::relocStatus()
{
    /**
     * 等待回复NAV_COMMAND_RELOC 3
     */
    /*
    if(time_cnt == 0)
    {
        ROS_INFO("write reloc");
        ROS_INFO("Restart with keyframe id: %d",followPath->getCurrentKeyFram());
        connent_tO_seslam->writerDataToSeslam(NAV_COMMAND_RELOC,followPath->getCurrentKeyFram());
    }
    wait();
    if(odom::getSeslamStatus() == SLAM_COMMAND_RELOC_SUCCESS && (odom::getSeslamInl() > SLAM_LOSE_INL))
    {
        ROS_INFO("get trials");
        localPlanState = follow_Path;
        time_cnt = 0;
    }
    */
}

/*
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    OK=2,
    LOST=3
*/

void Plan::SlamStateCallBack(const std_msgs::Int32& msg){
    static TrackingState lastSlamState = SYSTEM_NOT_READY;
    mSlamState = (TrackingState)msg.data;
    
    if(!mFollower->isHavePath()){
        isFirstGetSlamState = true;
    }
    
    if(isFirstGetSlamState){
        std_msgs::Bool goalMsg;
        goalMsg.data = true;
        goalCallPub.publish(goalMsg); 
        isFirstGetSlamState = false;
    }

    mSlamHeartbeat = 0;
    
#if 0
#if SIM_DEBUG == 0
    switch(mSlamState){
        case SYSTEM_NOT_READY:
        case NO_IMAGES_YET:
        case NOT_INITIALIZED:{
            localPlanState = stop;
        }
            break;
        case TRACKING_OK:{

            if(isFirstTracking){
                isFirstTracking = false;
                mLoseProcessing->setProcessingEnd(true);
                localPlanState = follow_Path;
            }else{
                if(lastSlamState == TRACKING_LOST){  //从lose状态回到tracking状态，证明找回了，所以需要重新定位，让车头大致朝向丢失时方向，问题是反复丢失如何找到
                    isCanUpdateYaw = false;
                    localPlanState = reloc;
                }else{ //tracking状态
                    if(localPlanState == reloc){ //重定位状态，这里要做的是,只有tracking时才能
                        if(mLoseProcessing->processingRelocal(mRobotLoseYaw, mRobotYaw)){

                        }
                    }else{

                    }
                }

            }

        }
            break;
        case TRACKING_LOST:{
            if(isCanUpdateYaw) {
                mRobotLoseYaw = mRobotYaw;
            }
            localPlanState = lose;
        }
        break;
        
    } 
#endif
#endif
    lastSlamState = mSlamState;


}

void Plan::Run(){

	std_msgs::String data;
	data.data = "hello";
	tp.publish(data);
	geometry_msgs::Twist tw;

	int pixel = 0; //mVisionDetect->getDetectPixelQuantity(9.0/10, mFarthestObstacle);

	std_msgs::Int32 pixelMsg;
	pixelMsg.data = pixel;

	mVisionPixelPub.publish(pixelMsg); 

	mSlamHeartbeat++;
	bool isSlamAlive = true;
	if(mSlamHeartbeat>20)
		isSlamAlive = false;

	static LocalPlanState oldState = idle;
	if(oldState != localPlanState){
		ROS_INFO("move state is %d", localPlanState);
	}
	oldState = localPlanState;
#if 1
	switch(localPlanState)
	{
		case idle:
			{
				Stop(tw);
				if(mSlamState == TRACKING_OK && mIsStartWork ){
					localPlanState = follow_Path;
				}
			}
			break;
		case follow_Path:
			{
#if DEBUG_PID  == 1
				outPutTwistMsg_ = followPath->debugPID(pidserver->pid_parameter);
#else
				//if(mFollower->robotFollowPath(tw) == false)
				//mIsToCharging = false;
				//mIsToReplenish = false;
				if(mFollower->robotFollowPath(tw, mIsStartWork, mIsToCharging, mIsToReplenish, mIsEnableLose, mIsEnableHaveObstaclesStopRobot) == false)
				{
#if SIM_DEBUG == 0
					stopStatus();
					Stop(tw);
#endif
					localPlanState = turn;
				}

				if((mSlamState == TRACKING_LOST) && mIsEnableLose){
					localPlanState = lose;
					break;
				}
				mRobotLoseYaw = mRobotYaw; //未丢失时更新
#endif
			}
			break;
		case turn:
			{
				Stop(tw);
				if(!mFollower->switchToNextGoal()){
					ROS_INFO("switchToNextGoal error");
				}else{
					localPlanState = follow_Path;
					ROS_INFO("switchToNextGoal ok");
				}

				/*
				   i f(mFollower->robotTurn(tw) == false)
				   {
				//#if SIM_DEBUG == 1
				localPlanState = follow_Path;
				//#else
				//localPlanState = stop;
				//#endif
				}
				*/
			}
			break;

		case stop:
			{
				stopStatus();
				Stop(tw);

			}
			break;
		case lose:
			{
				ROS_INFO("lose processing lose");
				mLoseProcessing->processingLose( odom::haveObstacles(),userScan,true,tw);
				if(mSlamState == TRACKING_OK){
					//localPlanState = reloc;
					localPlanState = follow_Path;
				}

				if(!mIsStartWork){
					localPlanState = follow_Path;
				}
			}
			break;
		case reloc:
			{
				//relocStatus();
				//Stop(tw);
				ROS_INFO("lose processing reloc goal = %f", mRobotLoseYaw);
				if(mSlamState == TRACKING_LOST){
					localPlanState = lose;
				}
				if(mLoseProcessing->processingRelocal(mRobotLoseYaw, mRobotYaw, tw)){
					localPlanState = follow_Path;
				}

				if(!mIsStartWork){
					localPlanState = follow_Path;
				}

			}
			break;

	}
#endif
#if 1 
	static int mode = 1;
	static int cnt = 0;
	static int cnt2 = 0;
    if (mIsTestTurn)
    {
        switch (mode)
        {
        case 1:
        {
            if (mFollower->turnAngleByWheelOdom(mTestTurnData, tw))
            {
                mode = 2;
            }
            break;
        }

        case 2:
        {
            if (mFollower->turnAngleByWheelOdom(-mTestTurnData, tw))
            {
                mode = 1;
                cnt++;
            }
            if (cnt == 20)
            {
                mIsTestTurn = false;
            }
            break;
        }
        default:
        {
            mode = 1;
        }
        }
        twistPub.publish(tw);
    }
    else
    {
        cnt = 0;
    }

    if (mIsTestRun)
    {
        switch (mode)
        {
        case 1:
        {
            if (mFollower->runByWheelOdom(mTestTurnData, tw))
            {
                //mode = 2;
                mIsTestRun = false;
            }
            break;
        }
        case 2:
        {

            if (mFollower->runByWheelOdom(-mTestTurnData, tw))
            {
                mode = 1;
                cnt2++;
            }

            if (cnt2 == 1)
            {
                mIsTestRun = false;
            }

            break;
        }

        default:
        {
            mode = 1;
        }
        }
        twistPub.publish(tw);
    }
    else
    {
        cnt2 = 0;
    }

#endif

			//ROS_INFO_STREAM(tw);

	if(isNavStatus ){
		//if((odom::haveObstacles() || pixel > mMaxObstaclePixel) && mIsEnableLose){
		if((odom::haveObstacles() ) && mIsEnableLose){
			ROS_INFO("have obstacles");
			Stop(tw);
		}

		//ROS_INFO("%d", mSlamHeartbeat);
		if(!isSlamAlive){
			ROS_ERROR("!!slam dead!!");
			Stop(tw);
		}

		twistPub.publish(tw);
	}
#ifdef USERSCAN
		if(userScan ){
			ROS_INFO("userScan ");	
			Stop(tw);
		}
#endif
		   //twistPub.publish(tw);
	}

void Plan::RvizGoalCallBack(const geometry_msgs::PoseStamped& msg)
{
    ROS_INFO("get goal");
    mIsGetRvizGoal = true;
    mRvizGoal = msg.pose;
    mIsStartWork = true;
}

void Plan::LaserDetectCallBack(const std_msgs::UInt8& msg)
{
    uchar data = msg.data;
    mIsSlowDown = false;
    mIsStop = false;
    
    if((data & 0x01)!=0){
        mIsSlowDown = true;
    }
    
    if((data & 0x02) != 0){
        mIsStop = true;
    }
}

void Plan::processStop(geometry_msgs::Twist& inOutTw,  const int& slowDown)
{
    if (mIsEnableLidarStop){
    if (mIsSlowDown)
    {
        inOutTw.angular.z = inOutTw.angular.z / 2.0;
        inOutTw.linear.x = inOutTw.linear.x / 2.0;
    }

    if (mIsStop)
    {
        {
            Stop(inOutTw);
            /*
            if (slowDown== 1)
            {
                Stop(inOutTw);
            }
            else if (slowDown == -2)
            {
                inOutTw.angular.z = 0.1;
                inOutTw.linear.x = inOutTw.linear.x / 2.0;
            }
            */
        }
        //else
        //{
        //    inOutTw.angular.z = inOutTw.angular.z / 2.0;
        //    inOutTw.linear.x = inOutTw.linear.x / 2.0;
        //}
    }}
}

void Plan::WriteLog(const int& runStatus,
                    const FPState::FollowPathState& currentState) {
    static int old_run_status = -1;
    static FPState::FollowPathState old_current_status = FPState::InitPath;

	if(old_run_status != runStatus){
        mLog->info("Send to server data: " + std::to_string(runStatus));
    }
    old_run_status = runStatus;

	if(old_current_status != currentState){
        mLog->info("Follow path state: " + std::to_string(currentState));
    }
    old_current_status = currentState;
}

void Plan::TestAmclRun()
{
    geometry_msgs::Twist tw;
    YoursNavPoint pointForStop;
    static int goalCnt = 0;
    float speed = 0.3;
    float stopWight = 0.9;
    float stopHeight = 1.5;
    int visionPixel = 0;
    int lidarDetectCode = 0;
    std_msgs::Float32MultiArray lidarDetectInfo;
    mFollower->getStopSpeedPara(speed,  stopWight,  stopHeight);

    if (mIsEnableHaveObstaclesStopRobotInt != -2)
    {
        mLidarDetect->setStopParam(stopWight, stopHeight);
		mVisionDetect->setFarthest(stopHeight - 0.7);
    }
    else
    {
        mLidarDetect->setStopParam(stopWight, 1.0);
        mVisionDetect->setFarthest(0.3);
    }
    //mLidarDetect->run(lidarDetectCode, lidarDetectInfo);
    mLidarDetect->run();

    bool isVisionDetectObstacles = false;
	isVisionDetectObstacles =   mVisionDetect->run(visionPixel);

	std_msgs::Int32 pixelMsg;
	pixelMsg.data = isVisionDetectObstacles;
	mVisionPixelPub.publish(pixelMsg); 

    static int soundCodeCnt = 0;
    /*
    if(mSoundCode.Code == 514){
        ROS_INFO_STREAM(" get 514, set Replenish false !!! ");
        mIsToReplenish = false;
        soundCodeCnt++;
    }
    if(soundCodeCnt > 10*5){
        mSoundCode.Code = 0;
        soundCodeCnt = 0;
    }
    */
    switch(localPlanState){
        case idle:{
            //必须地图加载完成后才能开始工作
            if (odom::getPoseReady()) {
                static unsigned int cnt = 0;
                if ((cnt % 50) == 0) {
                    mLog->info("AMCL load Map OK");
                }
                cnt++;
                if (mIsGetRvizGoal) {
                    localPlanState = follow_Path;
                    mIsGetRvizGoal = false;
                }
                if (isHaveAmclPath) {
                    localPlanState = follow_Path;
                }
            }else{
                mFollower->pubMapReadNotReady();
                static unsigned int cnt = 0;
                if ((cnt % 50) == 0) {
                    mLog->info("AMCL NOT LOAD MAP");
                }
                cnt++;
            }
            break;
        }
        case follow_Path:{
            bool t1 = true;
            //bool t2 = false;
            //bool t3 = false;
            bool t2 = true;
            bool t3 = true;
            bool t4 = true;
            bool t5 = false;
            if(mFollower->robotFollowPathAmcl(tw, mIsStartWork, mIsToCharging, mIsToReplenish , t4, mIsEnableHaveObstaclesStopRobotInt, mIsToReDock, mIsInitPath, t5, mTestCmd, mAMCLRunStatus, mFollowPathState)){
                
            }else{
                localPlanState = turn;
            }
            break;
        }
        case turn:{
            //if (mIsStartWork)
            //{
                if (mFollower->robotTurn(tw, mIsStartWork) == false)
                {
                    localPlanState = follow_Path;
                }
				if(!mIsStartWork){
            		Stop(tw);
				}
            //}
            break;
        }
        case reloc:{
            
            break;
        }

        case get_next_goal:
        {
            if (mPointMap.count(Yours_Sell_Path_1) > 0)
            {
                goalCnt++;
                if (goalCnt >= mPointMap.at(Yours_Sell_Path_1).size())
                {
                    goalCnt = 0;
                    pointForStop = mPointMap.at(Yours_Sell_Path_1)[mPointMap.at(Yours_Sell_Path_1).size() - 1];
                }
                else
                {
                    pointForStop = mPointMap.at(Yours_Sell_Path_1)[goalCnt - 1];
                }
                YoursNavPoint p = mPointMap.at(Yours_Sell_Path_1)[goalCnt];
                mRvizGoal.position.x = p.point.x;
                mRvizGoal.position.y = p.point.y;
                tf::Quaternion q = tf::createQuaternionFromYaw(p.point.yaw);
                mRvizGoal.orientation.w = q.w();
                mRvizGoal.orientation.x = q.x();
                mRvizGoal.orientation.y = q.y();
                mRvizGoal.orientation.z = q.z();
                localPlanState = follow_Path;
            }
            else
            {
                localPlanState = idle;
            }
            break;
        }
        default:{
            Stop(tw);
        }
    }

    WriteLog(mAMCLRunStatus, mFollowPathState);
    BackImpactRun(tw, mFollowPathState, is_back_impact_, mIsStartWork);

    static int mode = 1;
	static int cnt = 0;
	static int cnt2 = 0;
	if(mIsTestTurn){
		mIsEnableHaveObstaclesStopRobot = false;
		switch (mode){
			case 1:{
					   if(mFollower->turnAngleByWheelOdom(mTestTurnData, tw)){
						   mIsTestTurn = false;
						   //mode = 2;
					   }
					   break;
				   }

			case 2:{
					   if(mFollower->turnAngleByWheelOdom(-mTestTurnData, tw)){
						   mode = 1;
						   cnt++;
					   }
					   if(cnt == 20){
						   mIsTestTurn = false;

					   }
					   break;
				   }
			default:{
						mode = 1;
					}	
		}
	}else{
		cnt = 0;
	}
    //ROS_INFO("%d", mGoalPoseArray.poses.size());
    mGoalPoseArray.header.stamp = ros::Time::now();
    mGoalArrayPub.publish(mGoalPoseArray); 
    ///test
    if (mIsEnableHaveObstaclesStopRobotInt != -1)
    {
        processStop(tw, mIsEnableHaveObstaclesStopRobotInt);
    }
    if(isVisionDetectObstacles){
        Stop(tw);
    }
    static unsigned int uts_cnt = 0;
	/*
	if(odom::haveObstacles()){
		uts_cnt++;
		if((uts_cnt%100) == 0){
        mLog->info("Ultrasonic stop robot");
		}
        Stop(tw);
    }else{
		uts_cnt = 0;	
	}
	*/

    if(mArmErrorDetect->IsMotorError()){
        Stop(tw);
        mIsStartWork = false;
    }

    if(tw.linear.x < -0.01){
        //mLog->error(" Line Speed is Negative");
        //mLog->info(" Line Speed is Negative");
        std_msgs::Float64 msg;
        msg.data = tw.linear.x;
        mNavNegativePub.publish(msg);
        tw.linear.x = 0.0;
    }
    static double speed_c = 0.0;
    if (tw.linear.x > 0.1) {
        static unsigned int update_rate = 0;
        if ((update_rate % (10 * 5)) == 0) {
            double speed_cc = 0.0;
            int distance_ctrl = robot_distance_ctrl_->UpdateTwistByDistance(speed_cc, odom::getAmclPose());
            if (distance_ctrl == 1) {
                speed_c = speed_cc;
            } else if (distance_ctrl == 0) {
                speed_c = -2.0;
            } else {
                speed_c = 0.0;
            }
        }

        update_rate++;
        if (speed_c > -1.9) {
            tw.linear.x = tw.linear.x + speed_c;
        } else {
            Stop(tw);
        }
    }

    twistPub.publish(tw);
}

void Plan::RunRemap(){
    geometry_msgs::Twist tw;
	static LocalPlanState localPlanState_old = localPlanState;
 
    float speed = 0.3;
    float stopWight = 0.9;
    float stopHeight = 1.5;
    
	mFollower->getStopSpeedPara(speed,  stopWight,  stopHeight);
    mLidarDetect->setStopParam(stopWight, stopHeight);
	mLidarDetect->run();
    mVisionDetect->setFarthest(stopHeight - 0.7);
    int visionPixel = 0;
    bool isVisionDetectObstacles = mVisionDetect->run(visionPixel);
	
	switch(localPlanState){
		case idle:{
            if(isHaveAmclPath){
                localPlanState = follow_Path;
            }
            break;
        }
        case follow_Path:{
			if(!mFollower->RunRemapPath(tw, mIsStartWork)){
        		mLog->info("Plan To Turn Robot");
                localPlanState = turn;
            }	
			break;
		}
		case turn:{
			if(!mFollower->robotTurn(tw, mIsStartWork)){
        		mLog->info("Plan To Follow path");
                localPlanState = follow_Path;
            }
            break;
        }
        default:{
            break;
        }
    }

	if(localPlanState_old != localPlanState){
        mLog->info("Plan State: " + std::to_string(localPlanState));
        localPlanState_old = localPlanState;
    }

    mGoalPoseArray.header.stamp = ros::Time::now();
    mGoalArrayPub.publish(mGoalPoseArray); 

	if(!mIsStartWork){
        Stop(tw);
    }

    processStop(tw, true);

    if(isVisionDetectObstacles){
        Stop(tw);
    }

    twistPub.publish(tw);
}

/*
void msgToYoursNavPoint(const binorobot_msgs::YoursNavPoint&msg, YoursNavPoint& p){
     p = YoursNavPoint(msg.x, msg.y, msg.yaw, msg.speed, msg.sw, msg.sh, (YoursPath)msg.pathId, msg.id);
}
*/

void Plan::fillRandomMissionArray(const std::map<YoursPath, std::vector<YoursNavPoint>> &pointMap, std::vector<YoursPath> &pathArray)
{
    pathArray.clear();
    if (pointMap.count(Yours_Sell_Path_1) > 0)
    {
        pathArray.push_back(Yours_Sell_Path_1);
    }
    if (pointMap.count(Yours_Sell_Path_2) > 0)
    {
        pathArray.push_back(Yours_Sell_Path_2);
    }
    if (pointMap.count(Yours_Sell_Path_3) > 0)
    {
        pathArray.push_back(Yours_Sell_Path_3);
    }
    if (pointMap.count(Yours_Sell_Path_4) > 0)
    {
        pathArray.push_back(Yours_Sell_Path_4);
    }
    if (pointMap.count(Yours_Sell_Path_5) > 0)
    {
        pathArray.push_back(Yours_Sell_Path_5);
    }
    if (pointMap.count(Yours_Sell_Path_6) > 0)
    {
        pathArray.push_back(Yours_Sell_Path_6);
    }
    if (pointMap.count(Yours_Sell_Path_7) > 0)
    {
        pathArray.push_back(Yours_Sell_Path_7);
    }
    ROS_INFO("random mission size %d", pathArray.size());
}

#if 0
void Plan::YoursNavPointCallBack(const binorobot_msgs::YoursNavPointWrap& msg){
    //std::string path(msg.data);
    mGoalPoseArray.poses.clear();
    mPointMap.clear();
    
    for(int i = 0; i < msg.points.size(); i++){
        binorobot_msgs::YoursNavPointArray array = msg.points[i];
        std::vector<YoursNavPoint> pointArray;
        for(int j = 0; j < array.points.size(); j++){
            YoursNavPoint p;
            msgToYoursNavPoint(array.points[j], p);
            pointArray.push_back(p);
            //YoursN mPointMap.at(array.pathID).push_back(p);
        }
        mPointMap[YoursPath(array.pathID)] = pointArray;
    }
    
    
    fillRandomMissionArray(mPointMap, mRandomMissionArray);
    mFollower->updatePath(mPointMap, mRandomMissionArray);
    
    for(auto it = mPointMap.begin(); it != mPointMap.end(); it++){
        
        for(int i = 0; i < mPointMap.at(it->first).size(); i++ ){
            YoursNavPoint p = mPointMap.at(it->first)[i];
            geometry_msgs::Pose pose;
            pose.position.x = p.point.x;
            pose.position.y = p.point.y;
            tf::Quaternion q = tf::createQuaternionFromYaw(p.point.yaw);
            pose.orientation.w = q.w();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            mGoalPoseArray.poses.push_back(pose);
            //ROS_INFO_STREAM(pose);
        }
    }
    mFollower->zhihengUpdatePath(mGoalPoseArray);

    mDangerZone.clear();
    this->drawZone(mDangerZone, _yoursPubVector); 


    for(auto it:msg.zones){
        rZone oneZone;
        for(auto zonePoint:it.points){
            rPoint p;
            p.x = zonePoint.x;
            p.y = zonePoint.y;
            oneZone.points.push_back(p);
        }
        mDangerZone.push_back(oneZone);
    }
    this->drawZone(mDangerZone, _yoursPubVector); 
    mFollower->updateDangerzone(mDangerZone);

    std_msgs::Int32 checkMsg;
    checkMsg.data = 1;
    mYoursNavPointCheckPub.publish(checkMsg);

    isGetDangerZone = true;
}
#endif

bool Plan::readLidarNavPoint(const std::string &path, std::map<YoursPath, std::vector<YoursNavPoint>> &pointMap, std::vector<rZone> &zones)
{
	ROS_INFO_STREAM("debug 2" << path);
    cv::FileStorage fs(path, cv::FileStorage::READ);
	ROS_INFO_STREAM("debug 3");

	/*
    try
    {
        //fs = cv::FileStorage(path, cv::FileStorage::READ);
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
	*/
    pointMap.clear();
    zones.clear();

    bool result = false;
    if (fs.isOpened())
    {
        std::vector<int> pathIndex;
        fs["Path_Index"] >> pathIndex;
        for (int i = 0; i < pathIndex.size(); i++)
        {
            std::string pathNameStr = "Path_" + std::to_string(pathIndex[i]);
            int pointNumber = 0;
            fs[pathNameStr + "_Have_Points"] >> pointNumber;
            for (int pii = 0; pii < pointNumber; pii++)
            {
                std::string pointNameStr = pathNameStr + "_Point_" + std::to_string(pii);
                std::vector<float> points;
                fs[pointNameStr] >> points;
                YoursNavPoint p(points[0], points[1], points[2], points[3], points[4], points[5], YoursPath(pathIndex[i]), (int)points[6]);
                pointMap[YoursPath(pathIndex[i])].push_back(p);
                if (YoursPath(pathIndex[i]) == Yours_Sell_Path_1)
                {
                    result = true;
                }
            }
        }

        int dangerZoneNumber = 0;
        fs["DangerZone_have"] >> dangerZoneNumber;
        mDangerZone.clear();
        //清除显示
        if (dangerZoneNumber > 0)
        {
            //_yoursPolygonVector.clear();
            //_yoursPolygon.points.clear();
            //BinoRvizDraw::drawPolygon(_yoursPolygon, _yoursDangerZoneCurrentEdit);
            //BinoRvizDraw::drawPolygonArray(_yoursPolygonVector, _yoursPubVector);
            this->drawZone(mDangerZone, _yoursPubVector);
        }
        //更新显示
        for (int zone = 0; zone < dangerZoneNumber; zone++)
        {
            int polygonNumber = 0;
            fs["DangerZone_" + std::to_string(zone) + "_have_point"] >> polygonNumber;
            rZone polygon;
            for (int pCnt = 0; pCnt < polygonNumber; pCnt++)
            {
                std::vector<double> points;
                fs["DangerZone_" + std::to_string(zone) + "_point_" + std::to_string(pCnt)] >> points;
                rPoint p;
                p.x = points[0];
                p.y = points[1];
                polygon.points.push_back(p);
            }
            mDangerZone.push_back(polygon);
        }
        zones = mDangerZone;

        return result;
    }
    else
    {
        return result;
    }
}

void Plan::RunZH(){
	static int mode = 1;
	static int cnt2 = 0;
    mTestTurnData = 180;
    static LocalPlanState oldState = null_state;
#if 0
    switch (mode)
    {
    case 1:
    {
        if (mFollower->turnAngleByWheelOdom(mTestTurnData, tw))
        {
            //mode = 2;
            mIsTestRun = false;
        }
        break;
    }
    case 2:
    {

        if (mFollower->turnAngleByWheelOdom(-mTestTurnData, tw))
        {
            mode = 1;
            cnt2++;
        }

        if (cnt2 == 1)
        {
            mIsTestRun = false;
        }

        break;
    }

    default:
    {
        mode = 1;
    }
    }
    twistPub.publish(tw);
#endif

    geometry_msgs::Twist tw;
    YoursNavPoint pointForStop;
    
    static int goalCnt = 0;
    float speed = 0.3;
    float stopWight = 0.9;
    float stopHeight = 1.5;
    int visionPixel = 0;
    int lidarDetectCode = 0;
    std_msgs::Float32MultiArray lidarDetectInfo;

    //mLidarDetect->run(lidarDetectCode, lidarDetectInfo);
    mLidarDetect->run();

    if(oldState != localPlanState){
        if(localPlanState == idle){
            ROS_INFO_STREAM(" Plan State : IDLE");

        }else if(localPlanState == follow_Path){
            ROS_INFO_STREAM(" Plan State : FOLLOWPATH ");

        }else if(turn){
            ROS_INFO_STREAM(" Plan State : TURN ");

        }else{
            ROS_INFO_STREAM(" Plan State : " << localPlanState);

        }
    }
    oldState = localPlanState;

    switch(localPlanState){
        case idle:{
            if(mIsGetRvizGoal){
                mIsGetRvizGoal = false;
                mGoalPoseArray.poses.clear();
                mGoalPoseArray.poses.push_back(mRvizGoal);
                mFollower->zhihengUpdatePath(mGoalPoseArray);
                localPlanState = follow_Path;
            }
            
            if(isHaveAmclPath){
                localPlanState = follow_Path;
            }
            break;
        }
        case follow_Path:{
            
            bool t1 = false;
             
            if(mFollower->robotFollowPathZhiheng(tw, mIsStartWork, t1)){
                
            }else{
                localPlanState = turn;
            }

            if (mIsGetRvizGoal)
            {
                mIsGetRvizGoal = false;
                mGoalPoseArray.poses.clear();
                mGoalPoseArray.poses.push_back(mRvizGoal);
                mFollower->zhihengUpdatePath(mGoalPoseArray);
            }

            break;
        }
        case turn:{
            //ROS_INFO("turn");
            if(mFollower->robotTurn(tw, mIsStartWork) == false){
                localPlanState = follow_Path;
            }
            break;
        }
        case reloc:{
            
            break;
        }
        
        case get_next_goal:{
            if(mPointMap.count(Yours_Sell_Path_1) > 0){
                goalCnt++;
                if(goalCnt >= mPointMap.at(Yours_Sell_Path_1).size()){
                    goalCnt = 0;
                    pointForStop =mPointMap.at(Yours_Sell_Path_1)[mPointMap.at(Yours_Sell_Path_1).size() - 1]; 
                }else{
                    pointForStop =mPointMap.at(Yours_Sell_Path_1)[goalCnt - 1]; 
                }
                YoursNavPoint p = mPointMap.at(Yours_Sell_Path_1)[goalCnt];
                mRvizGoal.position.x = p.point.x;
                mRvizGoal.position.y = p.point.y;
                tf::Quaternion q = tf::createQuaternionFromYaw(p.point.yaw);
                mRvizGoal.orientation.w = q.w();
                mRvizGoal.orientation.x = q.x();
                mRvizGoal.orientation.y = q.y();
                mRvizGoal.orientation.z = q.z();
                localPlanState = follow_Path;
            }else{
                localPlanState = idle;
            }
            break;
        }
       
        default:{
            Stop(tw);
        }
        
    }
    //ROS_INFO("%d", mGoalPoseArray.poses.size());
    mGoalPoseArray.header.stamp = ros::Time::now();
    mGoalArrayPub.publish(mGoalPoseArray); 
    if(mIsEnableHaveObstaclesStopRobot){
        processStop(tw);
    }
    if(!mIsStartWork){
        Stop(tw);
    }
/*
    if(isVisionDetectObstacles){
        Stop(tw);
    }
    if(odom::haveObstacles()){
        Stop(tw);
    }
*/
    twistPub.publish(tw);
}

/*
void Plan::SoundCodeCallback(const yours_sound_play::sound_code &msg)
{   
    ROS_INFO_STREAM("get sound " << msg );
    mSoundCode = msg;
}
*/

float getYawGmsg( geometry_msgs::Pose  p1, geometry_msgs::Pose p2){
    float a = 0, Yaw = 0;
    a = -(p1.position.y - p2.position.y) / (p1.position.x - p2.position.x);
    if(std::isnan(a)){
        if(p1.position.y > p2.position.y)
            Yaw = -1.57;
        else
            Yaw = 1.57;
    }
    else{
        if(a < 0){
            Yaw = -atanf(a);
            if(p1.position.y > p2.position.y){
                if(Yaw > 0)
                    Yaw = Yaw + M_PI;
                else
                    Yaw = Yaw - M_PI;
            }
        }
        else{
            Yaw = -atanf(a);
            if(p1.position.y < p2.position.y){
                if(Yaw > 0)
                    Yaw = Yaw + M_PI;
                else
                    Yaw = Yaw - M_PI;
            }
        }
    }
    //geometry_msgs::Quaternion q;
    //q = tf::createQuaternionMsgFromYaw(Yaw);
    return Yaw;
}

void Plan::ZhihengPoseCallBack(const geometry_msgs::PoseArray &msg)
{
    if (!msg.poses.empty())
    {
        mGoalPoseArray.poses.clear();
        mGoalPoseArray.poses.push_back(msg.poses[0]);
        if (msg.poses.size() > 1)
        {

            for (int i = 1; i < msg.poses.size(); i++)
            {
                geometry_msgs::Pose p1 = msg.poses[i - 1];
                geometry_msgs::Pose p2 = msg.poses[i];
                double yaw = getYawGmsg(p1, p2);
                geometry_msgs::Quaternion q;
                q = tf::createQuaternionMsgFromYaw(yaw);
                p2.orientation = q;
                mGoalPoseArray.poses.push_back(p2);
                ROS_INFO_STREAM("pose : " << p2);
            }
            mGoalPoseArray.poses[0].orientation = mGoalPoseArray.poses[1].orientation;
        }
        mFollower->zhihengUpdatePath(mGoalPoseArray);
        isHaveAmclPath = true;
    }
}

void Plan::ZhihengStartWorkCallBack(const std_msgs::UInt8& msg)
{
    std_msgs::UInt8MultiArray msgPub;
    msgPub.data.push_back(msg.data);
    if (msg.data == 1)
    {
        mIsStartWork = true;
        if (!mGoalPoseArray.poses.empty())
        {
            msgPub.data.push_back(1);
        }
        else
        {
            msgPub.data.push_back(2);
        }
    }
    else if (msg.data == 2)
    {
        mIsStartWork = false;
        msgPub.data.push_back(1);
    }
    mZhihengStatePub.publish(msgPub);
}

void drawPolygon(const rZone& polygon, ros::Publisher& pub){
    geometry_msgs::PolygonStamped polygonMsg;
    polygonMsg.header.frame_id = "/map";
    polygonMsg.header.stamp = ros::Time::now();
    for (auto point : polygon.points)
    {
        geometry_msgs::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = 1.0;
        polygonMsg.polygon.points.push_back(p);
    }
    pub.publish(polygonMsg);
}

void Plan::drawZone(const std::vector<rZone> &zone, const std::vector<ros::Publisher> &pub)
{
    int cnt = 0;

    for (auto i : pub)
    {
        rZone willPub;
        if (!zone.empty())
        {
            if (cnt > zone.size() - 1)
            {
            }
            else
            {
                willPub = zone[cnt];
            }
        }

        drawPolygon(willPub, i);
        cnt++;
    }
}

void Plan::testDangerZone(){
    double startX = -17.0;
    double startY = -19.0;
    double endX = 72.0;
    double endY = 33.0;
    
    if(isGetDangerZone){
        ROS_INFO("test_get_zone");
        geometry_msgs::PoseArray array;
        array.header.frame_id = "map";
        array.header.stamp = ros::Time::now();
        int cnt = 0; 
        for (int y = startY; y < endY; y = y + 2)
        {
            for (int x = startX; x < endX; x = x + 2)
            {
                rPoint p;
                p.x = x;
                p.y = y;
                int zoneNum = 0;
                if(mFollower->isRobotInDangerZones(p, zoneNum)){
                    geometry_msgs::Pose pose;
                    pose.position.x = p.x;
                    pose.position.y = p.y;
                    ROS_INFO_STREAM(cnt);
                    array.poses.push_back(pose);
                    cnt++;
                    //break;
                }

                if(!mFollower->isRobotOutPath(p)){
                    geometry_msgs::Pose pose;
                    pose.position.x = p.x;
                    pose.position.y = p.y;
                    array.poses.push_back(pose);
                }
            }
        }



        _yoursDangerZoneTest.publish(array);
        isGetDangerZone = false;
    


    
    }

}

void Plan::MoveBaseCallBack(const move_base_msgs::MoveBaseActionResult& msg){
    mMoveResultMsg = msg;
}

void Plan::MoveBaseVelCallBack(const geometry_msgs::Twist &msg)
{
    mMoveBaseVelMsg = msg;
}


void Plan::testTebPlan(){
    geometry_msgs::Twist tw;
    static int _currentGoalNum = 0;
    static LocalPlanState oldState = null_state; 
    if(oldState != localPlanState){
        ROS_INFO_STREAM(" STATE : " << localPlanState);
    }
    oldState = localPlanState;
    static geometry_msgs::Pose currentGoal;  
    switch (localPlanState)
    {
    case idle:
    {
        if (mIsStartWork)
        {
            if (mGoalPoseArray.poses.size() > 0)
            {
                localPlanState = follow_Path;
                _currentGoalNum = 0;
                //geometry_msgs::Pose pose = mGoalPoseArray.poses[_currentGoalNum];
                currentGoal = mGoalPoseArray.poses[_currentGoalNum];
                mMoveBaseGoalMsg.goal.target_pose.header.stamp = ros::Time::now();
                mMoveBaseGoalMsg.goal.target_pose.pose = currentGoal;
                ROS_INFO_STREAM("------+++++");
                mMoveBaseGoalPub.publish(mMoveBaseGoalMsg);
            }
        }
        break;
    }

    case follow_Path:
    {
        tw = mMoveBaseVelMsg;
        nav_msgs::Odometry currentPose = odom::getAmclPose();
        rPoint p1(currentPose.pose.pose.position.x, currentPose.pose.pose.position.y);
        rPoint p2(currentGoal.position.x, currentGoal.position.y);
        double dist = p1.distanceTo(p2);
        
        if(dist < 0.3){

            localPlanState = get_next_goal;
           // mMoveResultMsg.status.status = 0;
        }
        if (mMoveResultMsg.status.status == 3)
        {
            localPlanState = get_next_goal;
            mMoveResultMsg.status.status = 0;
        }

        break;
    }
    case get_next_goal:
    {

        if (mGoalPoseArray.poses.size() > 1)
        {
            _currentGoalNum++;
            if (_currentGoalNum == mGoalPoseArray.poses.size())
            {
                _currentGoalNum = 0;
            }
            //geometry_msgs::Pose pose = mGoalPoseArray.poses[_currentGoalNum];
           currentGoal = mGoalPoseArray.poses[_currentGoalNum]; 
            mMoveBaseGoalMsg.goal.target_pose.header.stamp = ros::Time::now();
            mMoveBaseGoalMsg.goal.target_pose.pose = currentGoal;
            mMoveBaseGoalPub.publish(mMoveBaseGoalMsg);
            //准备pub 一个新的点给 move base
            localPlanState = follow_Path;
        }
        else
        {
            localPlanState = idle;
            mIsStartWork = false;
        }

        break;
    }
    default:
    {
        Stop(tw);
    }
    }

    if(!mIsStartWork){
        Stop(tw);
    }

    twistPub.publish(tw);
    mGoalArrayPub.publish(mGoalPoseArray); 
}

void Plan::UpdatePath(const std_msgs::Bool& msg){
    if(readLidarNavPoint(mNavPointPath, mPointMap, mDangerZone)){
        mGoalPoseArray.poses.clear(); //清除显示
        fillRandomMissionArray(mPointMap, mRandomMissionArray);
        mFollower->updatePath(mPointMap, mRandomMissionArray);
        for(auto it = mPointMap.begin(); it != mPointMap.end(); it++){
            for(int i = 0; i < mPointMap.at(it->first).size(); i++ ){
                YoursNavPoint p = mPointMap.at(it->first)[i];
                geometry_msgs::Pose pose;
                pose.position.x = p.point.x;
                pose.position.y = p.point.y;
                tf::Quaternion q = tf::createQuaternionFromYaw(p.point.yaw);
                pose.orientation.w = q.w();
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                mGoalPoseArray.poses.push_back(pose);
                ROS_INFO_STREAM(pose);
            }
        }

        this->drawZone(mDangerZone, _yoursPubVector);
        mFollower->updateDangerzone(mDangerZone);
        //声音能播放遇到障碍物
        
        isHaveAmclPath = true;
        mFollower->zhihengUpdatePath(mGoalPoseArray);
        mLog->info("Update Path success");
    } else {
        mLog->info("Update Path fail");
        ROS_INFO("amcl path read fail");
    }
}

void Plan::TestCMDCallBack(const std_msgs::Int32& msg){
    mTestCmd = msg.data;
}

void Plan::TestLocalPlan(){
        geometry_msgs::Twist tw;
    YoursNavPoint pointForStop;
    
    static int goalCnt = 0;
    float speed = 0.3;
    float stopWight = 0.9;
    float stopHeight = 1.5;
    int visionPixel = 0;
    int lidarDetectCode = 0;
    std_msgs::Float32MultiArray lidarDetectInfo;

    mFollower->getStopSpeedPara(speed,  stopWight,  stopHeight);
    mLidarDetect->setStopParam(stopWight, stopHeight);
    mLidarDetect->run();
    //mLidarDetect->run(lidarDetectCode, lidarDetectInfo);
    mVisionDetect->setFarthest(stopHeight);
    bool isVisionDetectObstacles = mVisionDetect->run(visionPixel); 
    static int soundCodeCnt = 0;

    //mFollower->testPubGlobalPath();
    /*
    if(mSoundCode.Code == 514){
        ROS_INFO_STREAM(" get 514, set Replenish false !!! ");
        mIsToReplenish = false;
        soundCodeCnt++;
    }

    if(soundCodeCnt > 10*5){
        mSoundCode.Code = 0;
        soundCodeCnt = 0;
    }
    */

    switch (localPlanState)
    {
    case idle:
    {
            if(isHaveAmclPath){
                localPlanState = follow_Path;
            }
        break;
    }
    case follow_Path:{
        if (!mFollower->robotFollowPathLocalPlanTest(tw, mIsStartWork))
        {
            localPlanState = turn;
        }

        break;
    }

    case turn:
    {
        ROS_INFO("turn");
        if (mFollower->robotTurn(tw, mIsStartWork ) == false)
        {
            localPlanState = follow_Path;
        }
        break;
    }

    default:
    {
        Stop(tw);
    }
    }
    twistPub.publish(tw);

}

void Plan::ImpactSensorCallback(const std_msgs::UInt8& msg){
	if((msg.data&0x01) != 0){
        ROS_INFO_STREAM("Get back impact");
        is_back_impact_ = true;
    }else{
        is_back_impact_ = false;
	}
}

void Plan::BackImpactRun(geometry_msgs::Twist& tw,
                         const FPState::FollowPathState& currentState,
                         const bool& is_back_impact,
						 const bool& is_work) {
    static int ImpactRunStatus = 0;
    switch (ImpactRunStatus) {
        case 0: {
			if((currentState == FPState::RunFromChargeToSellPath) || (currentState == FPState::RunFromReplenishToSellPath )){
				if(is_back_impact){
					if(is_work){
                        mLog->info("Back impact touch, run 0.2m.");
                        ImpactRunStatus = 1;
                    }
                }
            }
            break;
        }
        case 1: {
			if(mFollower->runByWheelOdom(0.2, tw)){
                mLog->info("Back impact touch run finsh");
                ImpactRunStatus = 0;
            }

            break;
        }
        default: {
            ImpactRunStatus = 0;
            break;
        }
    }
}

void Plan::RunChargeTest(){
    geometry_msgs::Twist tw;
    static int error_code = 0;
    
    mLidarDetect->run();

    switch(localPlanState){
        case idle:{
            if(isHaveAmclPath){
                localPlanState = follow_Path;
            }
            break;
        }
        case follow_Path:{
            if(!mFollower->runChargeTest(tw, mIsStartWork, error_code)){
                localPlanState = turn;
            }

            break;
        }
        case turn:{
            if (mIsStartWork) {
                if (mFollower->robotTurn(tw, mIsStartWork) == false) {
                    localPlanState = follow_Path;
                }
            }
            break;
        }
        default: {
            Stop(tw);
        }
    }

//    BackImpactRun(tw, FPState::RunFromChargeToSellPath , is_back_impact_, mIsStartWork);

    twistPub.publish(tw);
}
