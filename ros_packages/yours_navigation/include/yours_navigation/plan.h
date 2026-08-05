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

#ifndef PLAN_H
#define PLAN_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/CompressedImage.h>
#include <memory>
#include "followpath.h"
#include "odom.h"
#include "LoseProcessing.h"
#include "yours_vision_detect_node.h"
#include "yourslidardetect.h"
#include "yours_lidar_detect_human.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
//#include <binorobot_msgs/YoursNavPointWrap.h>
#include "robot.h"
//#include <yours_sound_play/sound_code.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "yours_detect_arm_error_code.h"
#include "yours_get_goal_robot_distance.h"
#include <yours_robot_tools/yours_log.h>
//#include "testpubgoalpose.h"

enum LocalPlanState
{
    null_state = -1,
    idle = 0,
    follow_Path,
    turn,
    stop,
    lose,
    reloc,
    get_next_goal
};

enum TrackingState{
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    TRACKING_OK=2,
    TRACKING_LOST=3
};


class Plan {
    ros::NodeHandle mN;
    ros::Publisher tp;
    ros::Publisher twistPub;
    ros::Publisher mPubRgbImage;
    ros::Publisher mPubMissionRecall;
    ros::Publisher mVisionPixelPub;
    ros::Subscriber slamStateSub;
    ros::Subscriber rgbSub;
    ros::Subscriber goalSub;
    ros::Subscriber scanCodeSub;
    ros::Subscriber robotPoseSub;
    ros::Subscriber startWorkSub;
    ros::Subscriber toChargeSub;
    ros::Subscriber toReplenishSub;
    ros::Subscriber testTurnSub;
    ros::Subscriber testRunSub;
    ros::Subscriber mSubMission;
    ros::Subscriber mRvizGoalSub;
    
    ros::Subscriber mLaserDetectSub;
   
    ros::Subscriber mYoursNavPointSub;
    ros::Publisher mYoursNavPointCheckPub;
    
    ros::Subscriber mZhihengPoseArraySub;
    ros::Subscriber mZhihengStartWorkSub;
    ros::Publisher  mZhihengStatePub;
    ros::Publisher  mNavNegativePub;

    ros::Subscriber mSoundCodeSub;
    ros::Subscriber mDangerZoneSub;
    ros::Subscriber mImpactSensorSub;

    void testPubGoalPose();
    std::shared_ptr<followpath> mFollower;
    std::shared_ptr<odom> mOdom;
    LocalPlanState localPlanState;
    std::shared_ptr<LoseProcessing> mLoseProcessing;

    std::shared_ptr<yours_vision_detect_node> mVisionDetect;
    //std::shared_ptr<YoursLidarDetect> mLidarDetect;
   	//std::shared_ptr<yours_lidar_detect_human::YoursLidarDetectHuman> lidar_detect
   	std::shared_ptr<yours_lidar_detect_human::YoursLidarDetectHuman> mLidarDetect;
    std::shared_ptr<yours_detect_arm_error_code::YoursDetectArmErrorCode>mArmErrorDetect;
    std::shared_ptr<yours_log::YoursLog> mLog;
    std::shared_ptr<YoursGetGoalRobotDistance> robot_distance_ctrl_;
    void Stop(geometry_msgs::Twist& inout_tw);

    void stopStatus();
    void relocStatus();

    TrackingState mSlamState;
    void SlamStateCallBack(const std_msgs::Int32& msg);
    void GoalCallBack(const std_msgs::Bool& msg);
    void ScanCodeCallBack(const std_msgs::Bool& msg);
    void RgbImageCallBack(const sensor_msgs::CompressedImageConstPtr& msg);
    void RobotPoseCallBack(const nav_msgs::OdometryConstPtr& msg);
    void StartWorkCallBack(const std_msgs::Bool& msg);
    void ToChargeCallBack(const std_msgs::Bool& msg);
    void ToReplenishCallBack(const std_msgs::Bool& msg);
    void TestTurnCallBack(const std_msgs::Int32& msg);
    void TestRunCallBack(const std_msgs::Int32& msg);
    void MissionCallBack(const std_msgs::Int32& msg);


    void RvizGoalCallBack(const geometry_msgs::PoseStamped& msg);
    void LaserDetectCallBack(const std_msgs::UInt8& msg);
    // void YoursNavPointCallBack(const binorobot_msgs::YoursNavPointWrap& msg);

    void ZhihengPoseCallBack(const geometry_msgs::PoseArray& msg);
    void ZhihengStartWorkCallBack(const std_msgs::UInt8& msg);
    // void SoundCodeCallback(const yours_sound_play::sound_code& msg);

    double mRobotYaw;
    double mRobotLoseYaw;  //丢失时的z轴角度

    bool isNavStatus;
    bool userScan;
    bool isFirstTracking;
    bool isCanUpdateYaw;

    bool mIsStartWork;  //自清除信号,信号使用者接收到后清除
    bool mIsToCharging;   // = false;
    bool mIsToReplenish;  // = false;
    bool mIsToReDock;
    bool mIsInitPath;

    bool mIsTestTurn;
    bool mIsTestRun;
    int mTestTurnData;
    float mRadianBuffer;

    bool mIsEnableLose;
    bool mIsEnableHaveObstaclesStopRobot;
    int mIsEnableHaveObstaclesStopRobotInt;
    bool isFirstGetSlamState;
    ros::Publisher goalCallPub;
    int mMaxObstaclePixel;
    int mFarthestObstacle;
    int mSlamHeartbeat;
    bool mIsAmclLocation;
    bool mIsGetRvizGoal;
    geometry_msgs::Pose mRvizGoal;
    geometry_msgs::PoseArray mGoalPoseArray;
    ros::Publisher mGoalArrayPub;
    bool mIsStop;
    bool mIsSlowDown;
   
    void processStop(geometry_msgs::Twist& inOutTw, const int& slowDown = 1 );
    void fillRandomMissionArray(const std::map<YoursPath, std::vector<YoursNavPoint> >&pointMap, std::vector<YoursPath>& pathArray);
    std::string mNavFilePath;
    std::map<YoursPath, std::vector<YoursNavPoint> > mPointMap;
    
	std::vector<YoursPath> mRandomMissionArray;
    bool isHaveAmclPath;
    
    bool mIsEnableLidarStop;

    //yours_sound_play::sound_code mSoundCode;

    //for display
    ros::Publisher _yoursDangerZone1;
    ros::Publisher _yoursDangerZone2;
    ros::Publisher _yoursDangerZone3;
    ros::Publisher _yoursDangerZone4;
    ros::Publisher _yoursDangerZone5;

    ros::Publisher _yoursDangerZone6;
    ros::Publisher _yoursDangerZone7;
    ros::Publisher _yoursDangerZone8;
    ros::Publisher _yoursDangerZone9;
    ros::Publisher _yoursDangerZone10;

    std::vector<ros::Publisher> _yoursPubVector;
    ros::Publisher _yoursDangerZoneTest;

    // 
    std::vector<rZone> mDangerZone;

    void drawZone(const std::vector<rZone>& zone, const std::vector<ros::Publisher>& pub);

    bool isGetDangerZone;

    move_base_msgs::MoveBaseActionResult mMoveResultMsg;
    ros::Subscriber mMoveSub;
    void MoveBaseCallBack(const move_base_msgs::MoveBaseActionResult& msg);

    geometry_msgs::Twist mMoveBaseVelMsg;
    ros::Subscriber mMoveBaseVelSub;
    void MoveBaseVelCallBack(const geometry_msgs::Twist& msg);
    move_base_msgs::MoveBaseActionGoal mMoveBaseGoalMsg;
    ros::Publisher mMoveBaseGoalPub;

    ros::Subscriber mUpdatePathSub;
    void UpdatePath(const std_msgs::Bool& msg);

    std::string mNavPointPath;
    std::string mRemapPathFilePath;

    ros::Subscriber mTestCmdSub;
    int mTestCmd;
    void TestCMDCallBack(const std_msgs::Int32& msg);

    int mAMCLRunStatus;
    FPState::FollowPathState mFollowPathState;

    bool isHaveAutoCharge;
    bool isHaveBuhuoAutoCharge;

    void WriteLog(const int& runStatus,
                  const FPState::FollowPathState& currentState
    );

    bool is_back_impact_;
    void ImpactSensorCallback(const std_msgs::UInt8& msg);
    void BackImpactRun(geometry_msgs::Twist& tw, const FPState::FollowPathState& currentState, const bool& is_back_impact, const bool& is_work);

 public:
    Plan(ros::NodeHandle& vi_n, const bool& is_remap = false);
    ~Plan();
    void Run();
    void TestAmclRun();
    void RunZH();
    void testDangerZone();
    ///对接Movebase的路径规划
    void testTebPlan();
    ///没有Movebase的路径规划
    void TestLocalPlan();
    void RunChargeTest();
    bool readLidarNavPoint(const std::string& path, std::map<YoursPath, std::vector<YoursNavPoint> >&points, std::vector<rZone>& zones);
    void RunRemap();
};

#endif // PLAN_H
    
 