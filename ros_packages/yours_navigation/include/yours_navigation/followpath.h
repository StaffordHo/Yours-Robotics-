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

#ifndef FOLLOWPATH_H
#define FOLLOWPATH_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "testpubgoalpose.h"
#include "PID.h"
#include "robot.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
//#include <yours_message/yours_battery_arm_msg.h>
//#include <kids100a_msgs/sensor.h>
#include <nav_msgs/Path.h>
#include <yours_msgs/YoursBatteryStatus.h>
#include <yours_robot_tools/yours_log.h>

#define PI                  (3.1415926f)
#define ANGLE_PI            (180.0)
#define TURN_SEPT1_ANGLE    (10)
#define TURN_SEPT2_ANGLE    (60)
#define REACH_GOAL_IDS      (0.3f)
#define TURN_MIN_LIMIT_ANGLE (1)
#define MAX_DELT_DIS        (0.4)

enum FollowPathError{
    
    
};

//考虑下车辆正在充电点和补货点的情况

namespace FPState{
    enum FollowPathState{
        InitPath = 0,
        WaitRunCommand,
        RunSellPath,
        RunFromSellToChargePath,
        RunFromChargeToSellPath,
        RunFromSellToReplenishPath,
        RunFromReplenishToSellPath,
        RunPrepareCharge,
        RunPrepareReplenish,
        Charging,
        AutoDock,
        ReturnAutoDockStartPoint,
        Replenish,
        SwitchNextGoal,
        PowerUpReDock,
        ErrorStopRobot,
        RunEndCmdPath,
        RunTest1Path,
        RunStartCmdPath,
        RunAvoidObstaclePath,
        RunSellReturnPath,
        RunPrepareSellReturnPath,
        RunWaitRobotBack, //for jd
        RunSell2Path //for jd
    };

    enum AvoidObstacleState{
        Init = 0,
        CalculatePath,
        RunPath,
        CheckPath,
        Calculate2edPath,
        CalculateReturnPath
    };
    
    enum ZhihengState{
        ZhihengInitPath = 0,
        ZhihengWaitRun,
        ZhihengRun
    };

}

//路径间的链接点， 去往充电，充电返回，去往补货，补货返回
enum PathLinkPoint{
    Sell2ChargePoint = 0,
    Charge2SellPoint,
    Sell2ReplenishPoint,
    Replenish2SellPoint
};

struct RunParam{
    RunParam():
    speed(0.3),
    stopHeight(1.5),
    stopWight(0.9)
    {};
    
    float speed;
    float stopWight;
    float stopHeight;
};

/**
 * @brief 用于描述避障功能每个任务点
 * 任务点有两种类型。
 * 1. 一个是目标点，作用是让车辆到达这个点，点有位置和旋转
 * 2. 一个是原地旋转，作用是控制车旋转到固定方向，旋转有左旋和右旋
 * 分开目标点和旋转，是为了精准控制车辆行动
 * 
*/
struct AvoidPathCell{
    ///构造函数
    AvoidPathCell():
    isTurn(false),
    turnAngle(.0)
    {};
    ///标识这个任务点是否是旋转点，true是旋转点，false是目标点
    bool isTurn;
    ///任务点目标点是代表目标点
    YoursNavPoint goalPoint;
    ///任务点是旋转点时控制旋转，正值左旋，负值右旋
    double turnAngle;

    void debugPrint(){
        if(isTurn){
            std::cout << " turn cell, angle : " << turnAngle << std::endl;
        }else{
            std::cout << " goal cell, x : " << goalPoint.point.x << " y: " << goalPoint.point.y << " yaw : " << goalPoint.point.yaw << std::endl;
        }

    }
};

class followpath
{
    void getNextPose(float &delt_Dis, bool& isChargeGoalPose, bool& isEndOfPath, bool isLoop);
    
    uint8_t follow_pose_step_;
    float linear_speed;
    float max_linear_speed;
    float turn_linear_speed;
    float situ_rotation_angle_speed;
    float delt_angle;
    
    float mDeltDis;
    
    int   current_key_fram;
    int   real_key_fram_;
    
    ros::NodeHandle mN;
    geometry_msgs::Twist twist_msg_;
    nav_msgs::Odometry  follow_nextPose_;
    
    float getGoStraightAngleSpeed(float deltAngle);
    float getFollowPathLinearSpeed(float deltAngle);

    std::shared_ptr<TestPubGoalPose> kfPose;
    std::shared_ptr<PID::PID> Pid;
    
    FPState::FollowPathState mRunState;
    std::map<PathLinkPoint, int> mSellPathToOthersLinkID;
    geometry_msgs::PoseArray mPoseArray;
    
    bool runProcessing(geometry_msgs::Twist &out_put_twist,  bool& isChargeGoalPose, bool& isEndOfPath, bool isLoop = false);
    
    ros::Publisher mPubRunState;
    ros::Publisher mAutodockCtrlPub;
    ros::Subscriber mAutodockStatusSub;
    ros::Subscriber mBatterySub;
    ros::Publisher mAutoDockErrorPub;
    ros::Publisher mOneloopPub;
    ros::Publisher mRobotToDockPub;

    ros::Subscriber mKidSub;

    std::map<YoursPath, std::vector<YoursNavPoint> > mPointMap;
    std::vector<YoursPath> mRandomMissionArray;
    
    std::pair<YoursPath, int> mCurrentPoint;
    
    void trySwitchSellToChargePath(geometry_msgs::Pose &goal, const bool& isReplenish_ = false);
    void trySwitchSellToReplenishPath(geometry_msgs::Pose &goal);
    void navPointToRosPose(const YoursNavPoint& nP, geometry_msgs::Pose& rP);
	yours_msgs::YoursBatteryStatus mBattry;
    //yours_message::yours_battery_arm_msg mBattry;
    void batteryCallback(const yours_msgs::YoursBatteryStatus& msg);
    //void kidCallBack(const kids100a_msgs::sensor& msg);

    RunParam mRunParam;
   
    bool mIsAmcl;
	float fpwrappi(float ang);
	float getAbsAngle(nav_msgs::Odometry currentPose, nav_msgs::Odometry nextKeyFram);

    /*避障的成员变量和方法*/
    bool mIsAvoidEnable;
    std_msgs::Float32MultiArray mLidarInfo; 
    ///避障任务的路径点序列，按照顺序排列，执行完一个后就将任务指向+1。在执行返回任务时需要反向执行这些任务点
    std::vector<AvoidPathCell> mAvoidPathVector;
    ///避障任务的执行状态机
    FPState::AvoidObstacleState mAvoidState;


    void setLidarDetectInfo(const std_msgs::Float32MultiArray& lidarInfo);
    void setAvoidEnable(bool isEnable_);
    void autodockStatusCallback(const std_msgs::Int32& msg);
    std::vector<AvoidPathCell> CalculateAvoidPath(const std_msgs::Float32MultiArray& lidarInfo_);
    std_msgs::Int32 mDockstatus; 
    std_msgs::Int32 autodockCtrlData;

    FPState::ZhihengState mZhihengRunState;
    geometry_msgs::Pose mZhihengGoal;

    bool isDockTest;

    double mPreChargeAngle;
    double mPreBuhuoAngle;

    bool isHaveAutoCharge;
    bool isHaveBuhuoAutoCharge;

    int mPowerUpPunStatus;
    bool isPlugIn;

    bool isRobotInThisZone(const rPoint& p, const rZone& zone);
    std::vector<rZone> mDangerZone;
    std::vector<rLine> mPathLine;

    void updataRobotToDock();

    double mPathDistance;

    //kids100a_msgs::sensor mKids;

    std::vector<std::pair<int, float>> mCmdPathEnd;
    std::vector<std::pair<int, float>> mCmdPathStart;
    int mCurrentCmdPathCnt;
    ros::Publisher mInitPosePub;
    bool mIsSellPathLoop;
    double mInitPose[3];

    //局部定位的话题与回调函数
    ros::Subscriber mLoaclPathSub;
    void localPathCallBack(const nav_msgs::Path& msg);
    bool mIsGetLocalPath;
    ros::Publisher mGlobalPathPub;
    nav_msgs::Path mLocalPath;
    nav_msgs::Path mGlobalPath;

    ros::Publisher mGoalPub;

    void updataGoal();

    //接收到充电或者补货后的路径
    std::vector<YoursNavPoint> mReturnPath;
    bool mIsForwardReturn;
    bool mIsReturnToCharge;

    std_msgs::Int32 mPubState;
    
	geometry_msgs::Pose mGoalPose;
	std::shared_ptr<yours_log::YoursLog> mLog;
	//运行状态存储
	void WriteRunState(const FPState::FollowPathState& _state, std::shared_ptr<yours_log::YoursLog>& _log);


 public:
    followpath(ros::NodeHandle& vi_n, std::shared_ptr<yours_log::YoursLog> _log);
    ~followpath();
    
    /** 机器人速度生成模块，其中包含任务状态机
        
        @param out_put_twist  返回速度控制
        @param isStartOrStopWork 是否开始工作，开始时根据自身判断如何行走，关闭时停止任务,需要函数关闭行走，因为可能到达充电区
        @param isToCharging 返回充电
        @param isToReplenish 返回补货点
    */
    bool robotFollowPath(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork, bool& isToCharging, bool& isToReplenish
    ,bool& isEnableLose 
    ,bool& isEnableHaveObstaclesStopRobot
    );
    //bool robotFollowPath(geometry_msgs::Twist &out_put_twist, bool isStartOrStopWork, bool& isToCharging, bool& isToReplenish);
    bool findNearestNavPose(int& vo_poseID );
    bool findNearestNavPose(YoursPath& pathName, int& locationInVector, bool isInit);
    
    float getTurnAngleSpeed(float deltAngle);
    float getDeltAngle(float current_angle);
    bool robotTurn(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork);

    /** @brief 机器人行走目标点切换为下一个
     *  因为视觉导航，避免机器人原地自旋，所以让机器人错过当前导航点时，转向下一个导航点，
     *  建议在需要旋转的地方触发
     * 
     */
    bool switchToNextGoal();
    bool turnAngleByWheelOdom(const double& angle, geometry_msgs::Twist &out_put_twist);
    bool runByWheelOdom(const double& distance, geometry_msgs::Twist &out_put_twist);
    bool isHavePath();
    //这两个函数还没有完成，需要补全
    bool isCharging();
    bool isReplenish();
    
    //test amcl
    bool runProcessing(geometry_msgs::Twist &out_put_twist,  bool& isChargeGoalPose,  const geometry_msgs::Pose&goal);
    
    bool robotFollowPathAmcl(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork, bool isToCharging, bool isToReplenish
    ,bool& isEnableLose 
    ,int& isEnableHaveObstaclesStopRobot
    ,bool& isReDock
    ,bool& isInitStatus
    ,bool& isRunTestPath
    ,int& cmdData
    ,int& runStatus
    ,FPState::FollowPathState &currentState
    );

    bool robotFollowPathZhiheng(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork, bool& isCleanPath); 
    void zhihengUpdatePath(const geometry_msgs::PoseArray& path); 
    
    void updatePath(const std::map<YoursPath, std::vector<YoursNavPoint> >& pointMap, const std::vector<YoursPath>& randomMissionArray);
    
    bool robotFollowPathLocalPlanTest(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork);
    
    void switchCurrentPathNextGoal(geometry_msgs::Pose& goal);
    
    void getStopSpeedPara(float& speed, float& stopWight, float& stopHeight);
    void updateDangerzone(const std::vector<rZone>& zone);
    bool isRobotInDangerZones( const rPoint& p, int& zoneNumber);
    bool isRobotOutPath(const rPoint& p);   
    void testPubGlobalPath();
    bool runChargeTest(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork, int& error_code);

	//原地旋转测试用接口
	bool IsNeedToGoal(YoursNavPoint goal, const double& dis_threshold);
    bool IsTurnOk(geometry_msgs::Twist &out_put_twist);
    bool IsRunOk(geometry_msgs::Twist &out_put_twist);

	//重新建图车辆行走接口
    bool RunRemapPath(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork);

    /**
     * @brief 
     * 给上层一个地图读取完成的指示，同时再读取未完成的时候发布703，地图未读取
     * @return true 
     * @return false 
     */

    bool isMapReadOk();

    void pubMapReadNotReady();
};

#endif // FOLLOWPATH_H
