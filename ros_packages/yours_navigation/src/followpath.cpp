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
 * 修改记录
 * 
 * 修改点：deltDis 局部变量修改为 mDeltDis
 * 作用：用于指向下一个目标点时更新距离数据
 * 
 * 
 * 
 * 
 */

#include "../include/yours_navigation/followpath.h"
#include "../include/yours_navigation/odom.h"
#include "../include/yours_navigation/macro.h"
#include "../include/yours_navigation/execute_robot_return.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <ctime>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//#define SIM_DEBUG 1

geometry_msgs::Twist stop()
{
    geometry_msgs::Twist tmep_twist;
    tmep_twist.linear.x = 0;
    tmep_twist.angular.z = 0;
    return tmep_twist;
}

geometry_msgs::Twist turn(float speed=0.1){
    geometry_msgs::Twist out;
    out.angular.x = 0.0;
    out.angular.x = 0.0;
    out.angular.z = speed;
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    return out;
}

geometry_msgs::Twist goForward(float speed=0.1){
    geometry_msgs::Twist out;
    out.angular.x = 0.0;
    out.angular.x = 0.0;
    out.angular.z = 0.0;
    out.linear.x = speed;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    return out;
}


float followpath::fpwrappi(float ang)
{
    while (ang > ANGLE_PI)
        ang -=(2 * ANGLE_PI);
    while (ang <= -ANGLE_PI)
        ang += (2 * ANGLE_PI);
    return ang;
}

float getAngleFromRadian(float radian)
{
    return((radian * ANGLE_PI) / PI);
}
float getRadianFormAngle(float angle)
{
    return ((angle / ANGLE_PI) * PI);
}

float followpath::getAbsAngle(nav_msgs::Odometry currentPose, nav_msgs::Odometry nextKeyFram)
{
    float deltX = 0;
    float deltY = 0;
    float angle = 0;
    deltX = nextKeyFram.pose.pose.position.x - currentPose.pose.pose.position.x;
    deltY = nextKeyFram.pose.pose.position.y - currentPose.pose.pose.position.y;
#if SIM_DEBUG == 0
    tf::Quaternion q(nextKeyFram.pose.pose.orientation.x, nextKeyFram.pose.pose.orientation.y,
                     nextKeyFram.pose.pose.orientation.z,nextKeyFram.pose.pose.orientation.w);
    double slam_yaw;
    double slam_pitch;
    double slam_roll;
    tf::Matrix3x3(q).getRPY(slam_roll, slam_pitch, slam_yaw);
	//ROS_INFO(" goal YAW = %f camera YAw = %f ", slam_yaw, odom::getSeslamPitch());
#endif



    if (deltX == 0 && deltY == 0) {
        return 0;
    } else {
        #if SIM_DEBUG == 1
        angle = (fpwrappi(getAngleFromRadian(atan2(deltY, deltX) - (odom::getOdomYaw()))));
        #else
		if(mIsAmcl){
			angle = (fpwrappi(getAngleFromRadian(atan2(deltY, deltX) - (odom::getAmclYaw()))));                                        //todo
		}else{
			angle = (fpwrappi(getAngleFromRadian(atan2(deltY, deltX) - (odom::getSeslamPitch()))));                                        //todo
		}
		//angle = (fpwrappi(getAngleFromRadian(slam_yaw - (odom::getSeslamPitch()))));                                        //todo
        #endif

		//std::cout<<" to goal: " <<  ( atan2(deltY, deltX) * ANGLE_PI) / PI << "robot Yaw " << (odom::getSeslamPitch() * ANGLE_PI) / PI << std::endl;


        return angle;
    }
}

float getDeltLength(nav_msgs::Odometry currentPose, nav_msgs::Odometry nextKeyFram)
{
    float deltX = 0;
    float deltY = 0;
    deltX = nextKeyFram.pose.pose.position.x - currentPose.pose.pose.position.x;
    deltY = nextKeyFram.pose.pose.position.y - currentPose.pose.pose.position.y;
    return (deltX * deltX) + (deltY * deltY);
}

float getDeltLength(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    float deltX = 0;
    float deltY = 0;
    deltX = p2.position.x - p1.position.x;
    deltY = p2.position.y - p1.position.y;
    return sqrt((deltX * deltX) + (deltY * deltY));
}

float getDeltLength(cv::Point2d p1, cv::Point2d p2){
    double deltX = 0;
    double deltY = 0;
    deltX = p2.x - p1.x;
    deltY = p2.y - p1.y;
    return sqrt((deltX*deltX) + (deltY*deltY));
}


static int pathEditResult = -1;

void readCmdPath(const ros::NodeHandle& n_ ,const std::string& name_, std::vector<std::pair<int, float>> & cmdPath_ ){
    int number;
    n_.param(name_+"_number", number, int(0));
    cmdPath_.clear();
    for (int i = 0; i < number; i++)
    {
        int type;
        float value;
        std::pair<int, float> data;
        n_.param(name_+ "_" + std::to_string(i) + "_type", data.first, int(0)); 
        n_.param(name_+ "_" + std::to_string(i) + "_value", data.second, float(0));
	ROS_INFO_STREAM(" 1st :" << data.first);     
	ROS_INFO_STREAM(" 2nd :" << data.second);     
     
	cmdPath_.push_back(data); 
    
    }
}

followpath::followpath(ros::NodeHandle& vi_n, std::shared_ptr<yours_log::YoursLog> _log):
follow_pose_step_(1)
,linear_speed(0.)
,max_linear_speed(0.4)
,turn_linear_speed(0.2)
,situ_rotation_angle_speed(0.4)
,current_key_fram(-1)
,real_key_fram_(0)
,delt_angle(0.)
,mRunState(FPState::InitPath)
,mDeltDis(0.0)
,mIsAmcl(true)
,mIsAvoidEnable(false)
,mZhihengRunState(FPState::ZhihengInitPath)
,isDockTest(false)
,mPowerUpPunStatus(220)
,isPlugIn(false)
,mPathDistance(1.0)
,mCurrentCmdPathCnt(0)
,mIsSellPathLoop(true)
,mIsGetLocalPath(false)
,mIsForwardReturn(false)
,mIsReturnToCharge(false)
,mLog(_log)
//,mPubState(220)
{
    mN = vi_n;
    kfPose = std::make_shared<TestPubGoalPose>(mN);
    Pid = std::make_shared<PID::PID>(mN);
    
    mPubRunState = mN.advertise<std_msgs::Int32>("/yours_run_state", 10);
    
    mAutodockCtrlPub = mN.advertise<std_msgs::Int32>("/yours_autodocl_ctrl",1);
    mAutodockStatusSub = mN.subscribe("/yours_autodocl_status", 1, &followpath::autodockStatusCallback, this);
    mBatterySub = mN.subscribe("/yours_base/battery_status", 1, &followpath::batteryCallback, this);  
    mAutoDockErrorPub = mN.advertise<std_msgs::Int32>("/yours_autodock_error_code",1);
    mOneloopPub = mN.advertise<std_msgs::Bool>("/yours_one_loop",1);
    mRobotToDockPub = mN.advertise<std_msgs::Float32>("/yours_robot_to_dock_distance", 1);
    mCurrentPoint.first = Yours_Charge_To_Sell_Path;
    mCurrentPoint.second = 0;
    //mKidSub = mN.subscribe("/kids100a", 1, &followpath::kidCallBack, this);
    mInitPosePub = mN.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    //local plan
    mLoaclPathSub = mN.subscribe("/yours_local_plan_result", 1, &followpath::localPathCallBack, this);
    mGlobalPathPub = mN.advertise<nav_msgs::Path>("/yours_global_wanto_plan",1);
    //goal pub for debug
    mGoalPub = mN.advertise<geometry_msgs::PoseStamped>("/yours_goal_debug",1);


	mN.param("max_linear_speed", max_linear_speed, 0.25f);	
	mN.param("turn_linear_speed", turn_linear_speed, 0.1f);	
	mN.param("situ_rotation_angle_speed", situ_rotation_angle_speed, 0.1f);	
	std::string modeStr;
	mN.param("locate_mode", modeStr,std::string("amcl"));
	mN.param("dock_test", isDockTest, false);
    mN.param("prepare_charge_angle", mPreChargeAngle, double(180.0));
    mN.param("prepare_buhuo_angle", mPreBuhuoAngle , double(180.0));

    mN.param("have_auto_charge", isHaveAutoCharge, false);
    mN.param("have_buhuo_auto_charge", isHaveBuhuoAutoCharge, false);
    mN.param("is_sell_path_loop", mIsSellPathLoop, true);
    readCmdPath(mN, "end_cmd_path", mCmdPathEnd);
    readCmdPath(mN, "start_cmd_path", mCmdPathStart);
    
    mN.param("init_x", mInitPose[0], double(0.0));
    mN.param("init_y", mInitPose[1], double(0.0));
    mN.param("init_a", mInitPose[2], double(0.0));

    if(modeStr.compare("amcl")!=0){
		mIsAmcl=false;
	}

    unsigned int seed = time(0);
    srand(seed);
   // for(int i = 0; i < 100; i++){
   //     std::cout << rand()%7 + 1 << std::endl;
   // }                             
    
}

followpath::~followpath()
{

}
void followpath::batteryCallback(const yours_msgs::YoursBatteryStatus& msg)
{
    mBattry = msg;
    static int checkPlug = 0;

/*	
    if (mBattry.current > 19000)
    {
        checkPlug++;
        //isPlugIn = true;
    }else{
        checkPlug = 0;
    }

    if(checkPlug > 20){
        checkPlug = 21;
        isPlugIn = true;
    }else{
        isPlugIn = false;
    }
*/
}
/*

void followpath::kidCallBack(const kids100a_msgs::sensor& msg){
    mKids = msg;
}
*/

float followpath::getTurnAngleSpeed(float deltAngle)
{
    if(deltAngle <= 0)                                                     //todo
    {
        return (situ_rotation_angle_speed);
    }else{
        return (-situ_rotation_angle_speed);
    }
}

float followpath::getDeltAngle(float current_angle)
{
    float delt_angle_t;
    delt_angle_t = fpwrappi(current_angle - delt_angle);
    //ROS_INFO("current_angle = %f, delt_angle = %f", current_angle, delt_angle);
    return delt_angle_t;
}


bool followpath::robotTurn(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork)
{
/*
        YoursNavPoint globalGoal = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second]; //这里有角度值
        nav_msgs::Odometry currentPose  = odom::getAmclPose();
        nav_msgs::Path globalPathPub;
        
        geometry_msgs::PoseStamped  p1;
        p1.pose = currentPose.pose.pose;
        globalPathPub.poses.push_back(p1);

        tf::Quaternion q;
        if (mPointMap.at(mCurrentPoint.first).size() == 2 && mCurrentPoint.second == 0)
        {
            q = tf::createQuaternionFromYaw(-globalGoal.point.yaw);
        }
        else
        {
            q = tf::createQuaternionFromYaw(globalGoal.point.yaw);
        }

        p1.pose.position.x = globalGoal.point.x;
        p1.pose.position.y = globalGoal.point.y;
        //ROS_INFO_STREAM("yaw  "  << globalGoal.point.yaw );
        p1.pose.orientation.x = q.getX();
        p1.pose.orientation.y = q.getY();
        p1.pose.orientation.z = q.getZ();
        p1.pose.orientation.w = q.getW();
        globalPathPub.poses.push_back(p1);
        
        mGlobalPathPub.publish(globalPathPub);
*/

    static float deltangle = 0;
    if(odom::getPoseReady() == false)
    {
        out_put_twist = stop();
        ROS_ERROR("not get pose");
        return true;
    }
#if SIM_DEBUG == 1
    deltangle = getDeltAngle((odom::getOdomYaw() * ANGLE_PI) / PI);
#else
	if(mIsAmcl){
		deltangle = getDeltAngle((odom::getAmclYaw() * ANGLE_PI) / PI);
	}else{
		deltangle = getDeltAngle((odom::getSeslamPitch() * ANGLE_PI) / PI);
	}
#endif
    //ROS_INFO("deltangle = %f",deltangle);
    if(fabs(deltangle) <= TURN_SEPT1_ANGLE)
    {
        ROS_INFO(" turn to followPath");
        return false;
    }
    out_put_twist.angular.z = getTurnAngleSpeed(deltangle);
    out_put_twist.linear.x = 0;

    if(mPubState.data != 111 && mPubState.data != 221){
        if(!isStartOrStopWork){
            mPubState.data = mPubState.data & 0xFFFE;
        }
    }

	mPubRunState.publish(mPubState);
	return true;
}

/** 对路径进行可执行性检查
    检查路径内是否有可执行的路线，
    FromSellingToCharging  0 
    FromChargingToSelling  1
    Selling                2
    FromSellingToReplenish 3
    FromReplenishToSelling 4
    返回和出发路线需要成对出现，售卖路线必须要有（吗？）
    
    @return  -1 充电路径不全  -2 补货路径不全  -3 没有售卖路径 0 正常5条路径  1 充电和售卖路径 2 补货和售卖路径 3 只有售卖路径
 
 */

int checkPath(const std::map<int, geometry_msgs::PoseArray>& path){
    int sellSize = path.at(2).poses.size();
    int sell2ChargeSize = path.at(0).poses.size();
    int charge2SellSize = path.at(1).poses.size();
    int sell2ReplenishSize = path.at(3).poses.size();
    int replenish2SellSize = path.at(4).poses.size();
    if(sellSize != 0 && sell2ChargeSize != 0 && charge2SellSize != 0 && sell2ReplenishSize != 0 && replenish2SellSize != 0 ){
        return 0;
    } 
        
    if(sellSize == 0){
        return -3;
    }else{
        if((sell2ChargeSize == 0 || charge2SellSize == 0) && (sell2ReplenishSize == 0 || replenish2SellSize == 0)){
            return 3;
        }else if((sell2ChargeSize != 0 && charge2SellSize != 0) && (sell2ReplenishSize == 0 || replenish2SellSize == 0) ){
            return 1;
        }else if((sell2ChargeSize == 0 || charge2SellSize == 0) && (sell2ReplenishSize != 0 && replenish2SellSize != 0) ){
            return -1;
        }
        
        return 3;
    }
}

float findLocationCheck_0(const std::map<int, geometry_msgs::PoseArray>& path, int checkPathResult, int& pathCode, int& poseCode, float& minDest){
    geometry_msgs::Pose p1;
    geometry_msgs::Pose p2;
    float dest = 0.0;
    p2 = odom::getORBPose().pose.pose;
    
    if(checkPathResult == 0){
        p1 = path.at(4).poses[0]; //从补货到售卖路径的第一个点
        //p2 = odom::getOdomPose().pose.pose;
    //    p2 = odom::getORBPose().pose.pose;
        dest = getDeltLength(p1, p2);
        if(dest < minDest){
            minDest = dest;
            pathCode = 4;
            poseCode = 0;
        }
    }

    if(checkPathResult == 0 || checkPathResult == 1){
        p1 = path.at(1).poses[0]; //从充电到售卖路径的第一个点
        //p2 = odom::getOdomPose().pose.pose;
    //    p2 = odom::getORBPose().pose.pose;
        dest = getDeltLength(p1, p2);
        if(dest < minDest){
            minDest = dest;
            pathCode = 1;
            poseCode = 0;
        }

		ROS_INFO("check path result = 0 1, dest = %f", dest);
    }
    
    
    int itCnt = 0;
    for(auto it:path.at(2).poses){
        p1 = it;
        dest = getDeltLength(p1, p2);
        if(dest < minDest){
            minDest = dest;
            pathCode = 2;
            poseCode = itCnt;
        }
        itCnt++;
    }
    return minDest;
}

/** 找到车辆起始点，并返回是否找到以及找到的路径和位置id
 * 
 * @param path 
 * @param pathEditResult checkPath
 * @param threshold
 * @param testCode 预留，只有充电或者补货路径，让车也能跑，todo
 * @param pathCode 获得的路径ID
 * @param poseCode 获得的位置ID
 * @return 是否找到了起始位置，找到了等待开始运行命令，没找到需要上报错误
 
 */
bool findLocation(const std::map<int, geometry_msgs::PoseArray>& path, int checkPathResult, double threshold, int testCode, int& pathCode, int& poseCode){
    //五条路径，车辆可能在任意位置上，而且可能是任意方向（例如去往充电桩或者从充电桩返回售卖点），在任意位置出现i7重启的情况还要再走么？
    //现在定义i7重启是一个严重问题，重启后只要不在售卖路径上就不再行走，也就是i7启动时车辆必须要在售卖路线上，或者存在充电或者补货路径时要在补货路径端点。
    //可以预留一个测试的接口，测试时在任何地方都可能出问题，所以再添加一个测试接口，用于告诉机器去哪里。
    float minDest = 999;
        //首先检查是否在充电开始点和补货开始点
    findLocationCheck_0(path, checkPathResult, pathCode ,poseCode, minDest);
    if(minDest < threshold){
        return true;
    }else{
        ROS_ERROR("findLocationCheck_0 error threshold = %f, minDest = %f", threshold, minDest);
        return false;
    }
}

int findNearestPoseInPoseArray(const geometry_msgs::PoseArray& array, const geometry_msgs::Pose& pose){
    float minDistance = 999;
    int returnID = 0;
    int cnt = 0;
    for(auto it:array.poses){
        float distance = getDeltLength(it, pose);
        if(distance < minDistance){
            minDistance = distance;
            returnID = cnt;
        }
        cnt++;
    }
   
   return returnID;
}

void findSellLinkPoint(const std::map<int, geometry_msgs::PoseArray>& path, int checkPath, std::map<PathLinkPoint, int>& linkMap){
    //0 全部路线，都要检查  1 只有充电路径，不检查补货
    linkMap.clear();
    if(checkPath == 0){
        for(auto it:path){
            if(it.first == 0 ){
                geometry_msgs::Pose p1 = it.second.poses[0];
                int linkID = findNearestPoseInPoseArray(path.at(2), p1); //path[2] 就是售卖路径,这里就是查找售卖路径和0号，也就是从售卖到充电路径的交点
                linkMap[Sell2ChargePoint] = linkID;
            }else if(it.first == 1 ){
                geometry_msgs::Pose p1 =it.second.poses[it.second.poses.size() - 1];
                int linkID = findNearestPoseInPoseArray(path.at(2), p1); //path[2] 就是售卖路径,这里查找从充电到售卖路径的链接点
                linkMap[Charge2SellPoint] = linkID;
            }else if(it.first == 3){
                geometry_msgs::Pose p1 = it.second.poses[0];
                int linkID = findNearestPoseInPoseArray(path.at(2), p1); //path[2] 就是售卖路径,这里就是查找售卖路径和0号，也就是从售卖到充电路径的交点
                linkMap[Sell2ReplenishPoint] = linkID;
            }else if(it.first == 4){
                geometry_msgs::Pose p1 =it.second.poses[it.second.poses.size() - 1];
                int linkID = findNearestPoseInPoseArray(path.at(2), p1); //path[2] 就是售卖路径,这里查找从充电到售卖路径的链接点
                linkMap[Replenish2SellPoint] = linkID;
            }
            
        }
        
        
    }else if(checkPath == 1){ //charge and sell path
        for(auto it:path){
            if(it.first == 0 ){
                geometry_msgs::Pose p1 = it.second.poses[0];
                int linkID = findNearestPoseInPoseArray(path.at(2), p1); //path[2] 就是售卖路径,这里就是查找售卖路径和0号，也就是从售卖到充电路径的交点
                linkMap[Sell2ChargePoint] = linkID;
            }else if(it.first == 1 ){
                geometry_msgs::Pose p1 =it.second.poses[it.second.poses.size() - 1];
                int linkID = findNearestPoseInPoseArray(path.at(2), p1); //path[2] 就是售卖路径,这里查找从充电到售卖路径的链接点
                linkMap[Charge2SellPoint] = linkID;
            }           
        }
    }else {
        linkMap.clear();
    } 
    
} 



//bool followpath::robotFollowPath(geometry_msgs::Twist& out_put_twist)
bool followpath::robotFollowPath(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork, bool& isToCharging, bool& isToReplenish
    ,bool& isEnableLose_
    ,bool& isEnableHaveObstaclesStopRobot_
)
{
    static int pathCode = 2;
    static int poseCode = 0;
   
    static bool isPrintState = true;
    static bool result = false;
    
    bool isChangeGoalPose_ = false;
    bool isEndOfPath_ = false;
    static std_msgs::Int32 pubState;
    
    isEnableLose_ = true;
    isEnableHaveObstaclesStopRobot_ = true;

    static double chargeTurnAngle = 180.0;

    //ROS_INFO("mRunState %d", mRunState);
    switch (mRunState){
		
        case FPState::InitPath:{
            pubState.data = 0;
            if(isPrintState){
            ROS_INFO(" FPState::InitPath ");
                isPrintState = false;
            }
            
            if(kfPose->isGetGoal){
                if(kfPose->mPoseArrayMap.size()!=0){
                    //mRunState = FPState::WaitRunCommand;
                    pathEditResult = checkPath(kfPose->mPoseArrayMap);
                    ROS_INFO(" path check result is %d ", pathEditResult);
                    if(pathEditResult >= 0){
                        mRunState = FPState::WaitRunCommand;
                        isPrintState = true;
                    }
                }
                kfPose->isGetGoal = false;
            }
            out_put_twist = stop();
            result = true;
            break;
        }
        case FPState::WaitRunCommand:{
            pubState.data = 0;
            if(isPrintState){
                ROS_INFO(" FPState::WaitRunCommand ");
                isPrintState = false;
            }
            findSellLinkPoint(kfPose->mPoseArrayMap, pathEditResult, mSellPathToOthersLinkID);
            for(auto it:mSellPathToOthersLinkID){
                std::cout << "mSellPathToOthersLinkID is " <<it.first << " " << it.second << std::endl;
            }
            
            if(isStartOrStopWork){
                //根据车辆所在位置和获得的路线情况判断车辆能否行动
                bool pathIsOk = findLocation(kfPose->mPoseArrayMap, pathEditResult, 2.0, 0, pathCode, poseCode);
                
                if(pathIsOk){
                    ROS_INFO("findLocation is ok");
                }else{
                    ROS_INFO("findLocation is Error");
                }
                
                ROS_INFO("isStartOrStopWork pathCode = %d, poseCode = %d", pathCode, poseCode);
                
                if(pathIsOk){
                    mPoseArray = kfPose->mPoseArrayMap.at(pathCode);
                    current_key_fram = poseCode -1 ; //这个与初始化时将 current_key_fram设置为-1对应
                    isPrintState = true;
                    pubState.data = 331;
                    if(pathCode == 2){ //从售卖路径开始
                        mRunState = FPState::RunSellPath;
                    }else if(pathCode == 1){ //从充电路径开始
                        mRunState = FPState::RunFromChargeToSellPath;
                    }else if(pathCode == 4){ //从补货路径开始
                        mRunState = FPState::RunFromReplenishToSellPath;
                    }
                }else{
                    //没有找到最近的路径点，报告错误 todo
                    
                }
            }
            out_put_twist = stop();
            result = true;
            break;
        }
        case FPState::RunFromChargeToSellPath:{
            
            if(isPrintState){
            ROS_INFO(" FPState::RunFromChargeToSellPath");
                isPrintState = false;
            }
            
            ROS_INFO("%d %d", current_key_fram, mPoseArray.poses.size());
            
            //运行，走完，调用链接点
            result = runProcessing(out_put_twist, isChangeGoalPose_, isEndOfPath_);
            if(isChangeGoalPose_){
                if(current_key_fram == mPoseArray.poses.size() - 1){//指向最后一个点，此时应该将路径切换为售卖路径
                    
                    int sellPoseCode = mSellPathToOthersLinkID.at(Charge2SellPoint);
                    mPoseArray = kfPose->mPoseArrayMap.at(2);
                    follow_nextPose_.pose.pose = mPoseArray.poses[sellPoseCode];
                    current_key_fram = sellPoseCode;
                    kfPose->pubTestNextPose(2, current_key_fram);
                    isPrintState = true;
                    mRunState = FPState::RunSellPath;
                }else{
                    kfPose->pubTestNextPose(1, current_key_fram);
                }
            }
            
            break;
        }
        case FPState::RunFromSellToChargePath:{
            static unsigned int turnCtrl = 0;
            
            pubState.data = 341;
            if(isPrintState){
            ROS_INFO(" FPState::RunFromSellToChargePath");
                isPrintState = false;
            }
            //运行，走完，进入充电进程，速度控制交给充电程序,暂时测试时交给充电，然后跳入从充电桩返回售卖
            result = runProcessing(out_put_twist, isChangeGoalPose_, isEndOfPath_);
            
            if(isEndOfPath_){
                isPrintState = true;
                mPoseArray = kfPose->mPoseArrayMap.at(1);

                chargeTurnAngle = 0 - chargeTurnAngle;

                mRunState =FPState::RunPrepareCharge;
                isEnableLose_ = false;
                break;
            }
            
            if(isChangeGoalPose_){
                kfPose->pubTestNextPose(0, current_key_fram);
            }
            
            break;
        }
        
        case FPState::RunPrepareCharge:{
            isEnableLose_ = false;
            pubState.data = 341;
            ROS_INFO("IN RunPrepareCharge");

            if(turnAngleByWheelOdom(mPreChargeAngle, out_put_twist)){
                //mRunState =FPState::RunFromChargeToSellPath;
                isStartOrStopWork = false;
                mRunState =FPState::Charging;
                current_key_fram = -1;
            }
            
            break;
        }
        
        case FPState::RunSellPath:{
            //运行，走到链接点
            
            pubState.data = 331;
            if(isPrintState){
            ROS_INFO(" FPState::RunSellPath");
                isPrintState = false;
            }
            
            
            result = runProcessing(out_put_twist, isChangeGoalPose_, isEndOfPath_, true);


            if(pathEditResult == 0){//all path 
                ROS_INFO(" pathEditResult == 0 ");
               
                if(isToCharging || isToReplenish){
                    if(isChangeGoalPose_){
                        int chargePoseCode = mSellPathToOthersLinkID.at(Sell2ChargePoint);
                        int replenishPoseCode = mSellPathToOthersLinkID.at(Sell2ReplenishPoint);
                        if(isToCharging){
                            if(current_key_fram == (chargePoseCode)){
                                //此时刚得到目标点的序号是售卖路线与重点路线的交叉点，这时需要将目标点的坐标点改为去充电路线的第一点。
                                //这样做为了避免到这个点再指向充电路径，而产生的原地旋转指令。
                                mPoseArray = kfPose->mPoseArrayMap.at(0);
                                current_key_fram = 0 ;
                                follow_nextPose_.pose.pose = mPoseArray.poses[0];
                                isToCharging = false; //clear 
                                mRunState = FPState::RunFromSellToChargePath;
                            }
                            break;
                        }//end if(isToCharging)
                        
                        if(isToReplenish){
                            if(current_key_fram == replenishPoseCode){
                                mPoseArray = kfPose->mPoseArrayMap.at(3);
                                current_key_fram = 0 ;
                                follow_nextPose_.pose.pose = mPoseArray.poses[0];
                                isToReplenish = false;
                                mRunState = FPState::RunFromSellToReplenishPath;
                            }
                        }//end if(isToReplenish)
                    }//end if(isChargeGoalPose_)
                }
            }else if(pathEditResult == 1){//charge and sell
                //ROS_INFO(" pathEditResult == 1 ");
                if(isChangeGoalPose_){
                    //只有充电和售卖路径的话不要看补货转换点
                    int chargePoseCode = mSellPathToOthersLinkID.at(Sell2ChargePoint);
                    //int replenishPoseCode = mSellPathToOthersLinkID.at(Sell2ReplenishPoint);
                    
                    //只有售卖路径和充电路径的情况下，收到补货或者返回充电的状态都会执行
                    
                    if(isToCharging || isToReplenish){
                        if(current_key_fram == (chargePoseCode)){
                            //此时刚得到目标点的序号是售卖路线与重点路线的交叉点，这时需要将目标点的坐标点改为去充电路线的第一点。
                            //这样做为了避免到这个点再指向充电路径，而产生的原地旋转指令。
                            mPoseArray = kfPose->mPoseArrayMap.at(0);
                            current_key_fram = 0 ;
                            follow_nextPose_.pose.pose = mPoseArray.poses[0];
                            kfPose->pubTestNextPose(0, current_key_fram);
                            isPrintState = true;
                            if(isToCharging){
                                isToCharging = false; //clear 
                                isToReplenish = false; //这里待讨论，我们在充电时是否要将补货标志也清除，理论上是的，因为充电意味要关机了
                                mRunState = FPState::RunFromSellToChargePath;
                            }else if(isToReplenish){
                                isToReplenish = false; //这里待讨论，我们在充电时是否要将补货标志也清除，理论上是的，因为充电意味要关机了
                                mRunState = FPState::RunFromSellToReplenishPath;
                            }
                            break;
                        }//end if(current_key_fram == (chargePoseCode))
                        else{
                            ROS_INFO("waiting run to switch to charging point");
                        }
                        
                    } //end if(isToCharging)
                    kfPose->pubTestNextPose(2, current_key_fram);
                }
            }else if(pathEditResult == 3){//only sell
                if(isChangeGoalPose_){
                    kfPose->pubTestNextPose(2, current_key_fram);
                }
            }else{
                ROS_ERROR("ERROR pathEditResult = %d ", pathEditResult);
            }//end if(pathEditResul;)
            
            break;
        }
        
        case FPState::RunFromSellToReplenishPath:
        {
            pubState.data = 351;
            if(isPrintState){
            ROS_INFO(" FPState::RunFromSellToReplenishPath");
                isPrintState = false;
            }
            //运行，走完，进入充电进程，速度控制交给充电程序,暂时测试时交给充电，然后跳入从充电桩返回售卖
            result = runProcessing(out_put_twist, isChangeGoalPose_, isEndOfPath_);
            
            if(isEndOfPath_){
                isPrintState = true;
                if(pathEditResult == 1){
                    mPoseArray = kfPose->mPoseArrayMap.at(1);
                }else{
                    mPoseArray = kfPose->mPoseArrayMap.at(4);
                }
                
                mRunState =FPState::RunPrepareReplenish;
                isEnableLose_ = false;
                break;
            }
            
            if(isChangeGoalPose_){
                if(pathEditResult == 1){
                    kfPose->pubTestNextPose(0, current_key_fram);
                }else{
                    kfPose->pubTestNextPose(3, current_key_fram);
                }
            }
            
            break;
            
        }
        
        case FPState::RunPrepareReplenish:{
            isEnableLose_ = false;
            pubState.data = 351;
            ROS_INFO("IN RunPrepareReplenish");
            
            if(turnAngleByWheelOdom(180, out_put_twist)){
                //mRunState =FPState::RunFromChargeToSellPath;
                isStartOrStopWork = false;
                mRunState =FPState::Replenish;
                current_key_fram = -1;
            }
            break;
        }
        
        case FPState::RunFromReplenishToSellPath:
        {
            if(isPrintState){
            ROS_INFO(" FPState::RunFromReplenishToSellPath");
                isPrintState = false;
            }
            
            //ROS_INFO("%d %d", current_key_fram, mPoseArray.poses.size());
            
            //运行，走完，调用链接点
            result = runProcessing(out_put_twist, isChangeGoalPose_, isEndOfPath_);
            if(isChangeGoalPose_){
                if(current_key_fram == mPoseArray.poses.size() - 1){//指向最后一个点，此时应该将路径切换为售卖路径
                    int sellPoseCode = 0;
                    if(pathEditResult == 1){
                        sellPoseCode  = mSellPathToOthersLinkID.at(Charge2SellPoint);
                    }else{
                        sellPoseCode = mSellPathToOthersLinkID.at(Replenish2SellPoint);
                    }
                    mPoseArray = kfPose->mPoseArrayMap.at(2);
                    follow_nextPose_.pose.pose = mPoseArray.poses[sellPoseCode];
                    current_key_fram = sellPoseCode;
                    kfPose->pubTestNextPose(2, current_key_fram);
                    
                    isPrintState = true;
                    mRunState = FPState::RunSellPath;
                }else{
                    if(pathEditResult == 1){
                        kfPose->pubTestNextPose(1, current_key_fram);
                    }
                    else{
                        kfPose->pubTestNextPose(4, current_key_fram);
                    }
                }
            }
            
            break;
        }
        
        case FPState::ErrorStopRobot:
        {
            break;
        }
        case FPState::Charging:
        {
            pubState.data = 110;
            isEnableLose_ = false; 
            if(isStartOrStopWork){
                mRunState =FPState::RunFromChargeToSellPath;
            }
            
            break;
        }
       
        case FPState::Replenish:
        {
			ROS_INFO("Replenish");
            pubState.data = 220;
            isEnableLose_ = false; 
            if(isStartOrStopWork){
                mRunState = FPState::RunFromReplenishToSellPath;
            }
            
            break;
        }
       
        default:{
            break;
        }
        
    }
    
    if(!isStartOrStopWork){
       //ROS_INFO("stop work");		
       out_put_twist = stop(); 
       //pubState.data = 000;
       isEnableLose_ = false; 
    }

    mPubRunState.publish(pubState);
    
    return result;
}

bool followpath::runProcessing(geometry_msgs::Twist &out_put_twist,  bool& isChargeGoalPose, bool& isEndOfPath, bool isLoop)
{
    //if(kfPose->mPoseArray.poses.size() == 0){
    if(mPoseArray.poses.size() == 0){
        out_put_twist = stop();
    }else{
        
        //static float deltDis = 0;
        static float real_deltDis= 0;
        static float deltangle = 0;
        if(odom::getPoseReady() == false)
        {
            out_put_twist = stop();
            ROS_ERROR("not get pose");
            return true;
        }
        
        int nearestID = 0;
        findNearestNavPose(nearestID);
        
        getNextPose(mDeltDis, isChargeGoalPose, isEndOfPath, isLoop);
        //getRealNextPose(real_deltDis);
        #if SIM_DEBUG == 1
        mDeltDis = getDeltLength(odom::getOdomPose(), follow_nextPose_);
        deltangle = getAbsAngle(odom::getOdomPose(), follow_nextPose_);
        //real_deltDis = getDeltLength(odom::getOdomPose(), real_nextPose);
        #else
        //deltDis = getDeltLength(odom::getRobotPose(), follow_nextPose_);
        //deltangle = getAbsAngle(odom::getRobotPose(), follow_nextPose_);
        mDeltDis = getDeltLength(odom::getORBPose(), follow_nextPose_);
        deltangle = getAbsAngle(odom::getORBPose(), follow_nextPose_);
        //real_deltDis = getDeltLength(odom::getRobotPose(), real_nextPose);
        #endif

		if(fabs(deltangle) > TURN_SEPT2_ANGLE){
			ROS_INFO("!!!!!!!!!!!!!!!!!!!!!");
			ROS_INFO("only TURN_SEPT2_ANGLE");
			ROS_INFO("!!!!!!!!!!!!!!!!!!!!!");
		}

        if(fabs(deltangle) > TURN_SEPT2_ANGLE && mDeltDis < 1)
        {
            #if SIM_DEBUG == 1
            delt_angle = deltangle + (odom::getOdomYaw() * ANGLE_PI) / PI;
            #else
            delt_angle = deltangle + (odom::getSeslamPitch() * ANGLE_PI) / PI;
            #endif
            ROS_INFO("turn  delt_angle = %f, yaw = %f", deltangle, (odom::getOdomYaw()* ANGLE_PI) / PI);
            return false;
        }
        //        ROS_INFO("current_key_fram = %d, deltDis = %f, deltAngle = %f nearestID = %d ",real_key_fram_, real_deltDis, deltangle, nearestID);
        out_put_twist.angular.z = getGoStraightAngleSpeed(deltangle);
        out_put_twist.linear.x = getFollowPathLinearSpeed(deltangle);
        //ROS_INFO("current_key_fram = %d, deltDis = %f, deltAngle = %f nearestID = %d angle speed=%f ", current_key_fram, mDeltDis, deltangle, nearestID, out_put_twist.angular.z);
    }
    return true;
}

bool followpath::runProcessing(geometry_msgs::Twist &out_put_twist,  bool& isChargeGoalPose, const geometry_msgs::Pose&goal){
    
    //ROS_INFO_STREAM("my " << odom::getAmclPose().pose.pose);
    //ROS_INFO_STREAM("goal " << goal);
    
    nav_msgs::Odometry goal_;
    goal_.pose.pose = goal;
    mDeltDis = getDeltLength(odom::getAmclPose(), goal_);
    float deltangle = getAbsAngle(odom::getAmclPose(), goal_);
    if(fabs(deltangle) > TURN_SEPT2_ANGLE){
        ROS_INFO("!!!!!!!!!!!!!!!!!!!!!");
        ROS_INFO("only TURN_SEPT2_ANGLE %f", deltangle);
        ROS_INFO("!!!!!!!!!!!!!!!!!!!!!");
        delt_angle = deltangle + (odom::getAmclYaw() * ANGLE_PI) / PI;
        return false;
    }
    
    isChargeGoalPose = false;
    if(mDeltDis < REACH_GOAL_IDS){
        isChargeGoalPose = true;
    }
    
    
    if(fabs(deltangle) > TURN_SEPT2_ANGLE && mDeltDis < 1)
    {
        #if SIM_DEBUG == 1
        delt_angle = deltangle + (odom::getOdomYaw() * ANGLE_PI) / PI;
        #else
        delt_angle = deltangle + (odom::getAmclYaw() * ANGLE_PI) / PI;
        #endif
        ROS_INFO("turn  delt_angle = %f, yaw = %f", deltangle, (odom::getOdomYaw()* ANGLE_PI) / PI);
        return false;
    }
    
    out_put_twist.angular.z = getGoStraightAngleSpeed(deltangle);
    out_put_twist.linear.x = getFollowPathLinearSpeed(deltangle);
    
    
    return true;
}



void followpath::getNextPose(float &delt_Dis,  bool& isChangeGoalPose, bool& isEndOfPath, bool isLoop)
{
    isChangeGoalPose = false;
    isEndOfPath = false;
    kfPose->pubArray();
    nav_msgs::Odometry centerPose;
    if(delt_Dis <= REACH_GOAL_IDS)
    {
        ROS_INFO("----get new pose------");
        current_key_fram += follow_pose_step_;
        isChangeGoalPose = true;
        //if(current_key_fram >= kfPose->mPoseArray.poses.size())
        if(current_key_fram >= mPoseArray.poses.size())
        {
            ROS_INFO("----get last pose------");
            current_key_fram = 0;
            real_key_fram_ = 0;
            twist_msg_ = stop();
            delt_Dis = 0;
            isEndOfPath = true;
            if(!isLoop){
                return;
            }
        }
        #if SIM_DEBUG == 1
        //            kfPose->pubCurrentPose(odom::getOdomPose());
        #else
        //            kfPose->pubCurrentPose(odom::getRobotPose());
        #endif
        /*
         *            if(current_key_fram + 3 < kfPose->KFPoseBuff.size())
         *            {
         *                for(int i = 0; i < 3; i++)
         *                {
         *                    follow_Pose_buff_[i] = kfPose->KFPoseBuff[current_key_fram + i];
         *                }
         *            }
         */
        //geometry_msgs::Pose cp = kfPose->mPoseArray.poses[current_key_fram];
        
         geometry_msgs::Pose cp = mPoseArray.poses[current_key_fram];
         follow_nextPose_.pose.pose = cp;
         follow_nextPose_.header.stamp = ros::Time::now();
    }
}

/**
 * model == 1
 * deltAngle >= 0  turn left; deltAngle < 0  turn right
 * @param deltAngle
 * @param model
 * @return
 */
float followpath::getGoStraightAngleSpeed(float deltAngle)
{
    if(fabs(deltAngle) < TURN_MIN_LIMIT_ANGLE)
    {
        Pid->pid_set_param_limit_out( OUTPUT_LOW, OUTPUT_UPPER);
    }else{
        Pid->pid_set_param_limit_out( SLOW_OUTPUT_LOW, SLOW_OUTPUT_UPPER);
    }
    return (Pid->get_pid_turn_angle_speed(deltAngle));                     //todo
}

float followpath::getFollowPathLinearSpeed(float deltAngle)
{
    if(fabs(deltAngle) < TURN_MIN_LIMIT_ANGLE)
    {
        linear_speed +=0.04;
        if(linear_speed >= max_linear_speed)
        {
            linear_speed = max_linear_speed;
        }
    }else{
        linear_speed = turn_linear_speed;
    }
    return (linear_speed);
}

bool followpath::findNearestNavPose(int& vo_poseID)
{
    int mId = 0;
#if SIM_DEBUG == 1
    nav_msgs::Odometry currentOdom = odom::getOdomPose();
    
    float minDest = 999.0;
    int id = 0;
    for(auto buffPose: kfPose->mPoseArray.poses){
        nav_msgs::Odometry buffOdom;
        buffOdom.pose.pose = buffPose;
        float cD = getDeltLength(currentOdom, buffOdom);
        if(cD < minDest){
            minDest = cD;
            mId = id;
        }
        id++;
    }
    
    
    nav_msgs::Odometry findMin;
    findMin.pose.pose = kfPose->mPoseArray.poses[mId];
    int mIdNext = mId + 1;
    nav_msgs::Odometry findMinNext;
    if(mId != (kfPose->mPoseArray.poses.size() - 1)){
        findMinNext.pose.pose = kfPose->mPoseArray.poses[mId + 1];
    }
    else{
        ROS_INFO("CLEAR!!!!");
        findMinNext.pose.pose = kfPose->mPoseArray.poses[0];
        mIdNext = 0;
    }
    
    float distanceFromCurrentToMin = getDeltLength(currentOdom, findMin);
    float distanceFromCurrentToMinNext = getDeltLength(currentOdom, findMinNext);
    float distanceFromMinToMinNext = getDeltLength(findMin, findMinNext);
    if(distanceFromMinToMinNext > distanceFromCurrentToMinNext)
        mId = mIdNext;
    
    vo_poseID = mId;
#else
#endif

    return true;
}


/*
    FromSellingToCharging  0 
    FromChargingToSelling  1
    Selling                2
    FromSellingToReplenish 3
    FromReplenishToSelling 4
 */

bool followpath::switchToNextGoal()
{
    current_key_fram++;
    
    bool updateGoal = true;
    
    switch (mRunState){
        case FPState::RunSellPath:{
            if(current_key_fram >= kfPose->mPoseArrayMap.at(2).poses.size()){
                current_key_fram = 0;
            }
            //更新目标点和距离数据 
            
            break;
        }
        case FPState::RunFromChargeToSellPath:{
            //从充电到售卖路径，这最后一个点就是链接点，这样的话就要切换到售卖路径链接点的下一个点
            if(current_key_fram >= kfPose->mPoseArrayMap.at(1).poses.size()){
                int goalInSellPath = mSellPathToOthersLinkID.at(Charge2SellPoint);
                current_key_fram = goalInSellPath+1;
                if(current_key_fram >= kfPose->mPoseArrayMap.at(2).poses.size()){
                    current_key_fram = 0;
                }
                mPoseArray = kfPose->mPoseArrayMap.at(2);
            }
            break;
        }
        case FPState::RunFromReplenishToSellPath:{

			if(pathEditResult == 1){
				if(current_key_fram >= kfPose->mPoseArrayMap.at(1).poses.size()){
					int goalInSellPath = mSellPathToOthersLinkID.at(Charge2SellPoint);
					current_key_fram = goalInSellPath+1;
					if(current_key_fram >= kfPose->mPoseArrayMap.at(2).poses.size()){
						current_key_fram = 0;
					}
					mPoseArray = kfPose->mPoseArrayMap.at(2);
				}

			}else{
				//从补货到售卖路径，这最后一个点就是链接点，这样的话就要切换到售卖路径链接点的下一个点
				if(current_key_fram >= kfPose->mPoseArrayMap.at(4).poses.size()){
					int goalInSellPath = mSellPathToOthersLinkID.at(Replenish2SellPoint);
					current_key_fram = goalInSellPath+1;
					if(current_key_fram >= kfPose->mPoseArrayMap.at(2).poses.size()){
						current_key_fram = 0;
					}
					mPoseArray = kfPose->mPoseArrayMap.at(2);
				}
			}

            break;
        }
        case FPState::RunFromSellToChargePath:{
            if(current_key_fram >= kfPose->mPoseArrayMap.at(0).poses.size()){
                ROS_INFO(" switchToNextGoal  RunFromSellToChargePath");
                current_key_fram--;
                return false;
            }
            break;
        }
        case FPState::RunFromSellToReplenishPath:{
			if(pathEditResult == 1){
				if(current_key_fram >= kfPose->mPoseArrayMap.at(0).poses.size()){
					ROS_INFO(" switchToNextGoal  RunFromSellToReplenishPath");
					current_key_fram--;
					return false;
				}
			}else{

				if(current_key_fram >= kfPose->mPoseArrayMap.at(3).poses.size()){
					ROS_INFO(" switchToNextGoal  RunFromSellToReplenishPath");
					current_key_fram--;
					return false;
				}
			}
            break;
        }
        default:{
            updateGoal = false;
            break;
        }
    }
    
    if(updateGoal){
        geometry_msgs::Pose cp = mPoseArray.poses[current_key_fram];
        follow_nextPose_.pose.pose = cp;
        follow_nextPose_.header.stamp = ros::Time::now();
        mDeltDis = getDeltLength(odom::getORBPose(), follow_nextPose_);
    }
    return true;
}


bool isRadianSameSign(float r1, float r2){
    if(r1 > 0.0 && r2 > 0.0){
        return true;
    }else if(r1 < 0.0 && r2 < 0.0){
        return true;
    }else{
        return false;
    }
}

/**
 * @brief 
 * 
 * 
 * @return true转到了，false继续转
 */
bool followpath::turnAngleByWheelOdom(const double& angle, geometry_msgs::Twist &out_put_twist)
{
    double currentRadian =  (odom::getOdomYaw());
    
    static int stateM = 0;
    static float delta = 0.0;
    static float lastData = 0.0;
    
//    ROS_INFO("current angle = %f, initYaw_ = %f, state = %d, delta %f ", getAngleFromRadian(currentRadian), getAngleFromRadian(initYaw_), stateM, delta);
//    ROS_INFO("current angle = %f, state = %d, delta %f ,goal angle %f", getAngleFromRadian(currentRadian) , stateM, delta * 180.0/M_PI, angle);
    switch (stateM){
        case 0:{
            delta = 0;
            lastData = currentRadian;
            stateM = 1;
            break;  
        }
        case 1:{
            if(isRadianSameSign(currentRadian, lastData)){
  //              ROS_INFO("same sign");
                delta += fabs(currentRadian - lastData);
            }else{
 //               ROS_INFO("not same sign");
                if(angle > 0){
                    if(currentRadian > 0){
                        delta += fabs(currentRadian - lastData);
                    }else{
                        float temp = (PI - lastData) + (PI+currentRadian);
                        delta += temp;
                    }
                }else{
                    
                    if(currentRadian > 0){
                        float temp = (PI - currentRadian) + (PI+lastData);
                        delta += temp;
                    }else{
                        delta += fabs(lastData - currentRadian);
                    }
                    
                }
            }
            lastData = currentRadian;
            
            if(getAngleFromRadian(delta) > fabs(angle)){
                out_put_twist = stop();
                stateM = 0;
                ROS_INFO("turn OK");
                return true;
            }else{
                if(angle > 0.0)
                    out_put_twist = turn(0.3);
                else
                    out_put_twist = turn(-0.3);
                return false;
            }
        }
        default:{
            out_put_twist = stop();
            ROS_ERROR(" turnAngleByWheelOdom state error ");
        }
    }
    
    return false;
}


bool followpath::runByWheelOdom(const double& distance, geometry_msgs::Twist &out_put_twist){
    nav_msgs::Odometry currentOdom = odom::getOdomPose();
    cv::Point2d currentPoint = cv::Point2d(currentOdom.pose.pose.position.x, currentOdom.pose.pose.position.y);
    
    static int stateM = 0;
    static float delta = 0.0;
    static cv::Point2d lastPoint(0.0, 0.0);
    
    ROS_INFO("state = %d, delta %f ", stateM, delta);
    switch (stateM){
        case 0:{
                lastPoint = currentPoint;
                stateM = 1;
            break;
        }
        case 1:{
            delta = getDeltLength(currentPoint, lastPoint);
            if(delta > fabs(distance)){
                out_put_twist = stop();
                stateM = 3;
                return true;
            }else{
                if(distance > 0)
                out_put_twist = goForward(0.1);
                else
                out_put_twist = goForward(-0.1);
            }
            
            break;
        }
        case 3:{
            stateM = 0;
			delta = 0;
            break;
        }
        default:{
            out_put_twist = stop();
            ROS_ERROR("runByWheelOdom stateM error");
        }
        
        
    }
    
    
    return false;
    
}


bool followpath::isHavePath()
{
if(!mIsAmcl){
	if(kfPose->mPoseArrayMap.count(2) != 0){
        if(kfPose->mPoseArrayMap.at(2).poses.size() > 0){
            return true;
        }
    }
}else{
	if(!mRandomMissionArray.empty()){
		return true;
	}
}
    return false;
}

bool followpath::isCharging()
{
    if(mRunState == FPState::Charging){
        return true;
    }
    return false;
}

bool followpath::isReplenish()
{
    if(mRunState == FPState::Replenish){
        return true;
    }
    return false;
}

bool followpath::findNearestNavPose(YoursPath& pathName, int& locationInVector, bool isInit){
    nav_msgs::Odometry robotPose = odom::getAmclPose();
    std::pair<std::pair<int, YoursNavPoint>,  std::pair<int, YoursNavPoint>> yours_line;
    double minDist = 999.0;
    double minLineToPointDist = 999.0;
    bool result = false;
    for(auto it = mPointMap.begin(); it != mPointMap.end(); it++){
        int i = 0;
        int ii = 0;

        if(isInit){
            if(it->first == Yours_Sell_Path_1 || it->first == Yours_Charge_To_Sell_Path || it->first == Yours_Replenish_To_Sell_Path){
                
            }else{
                continue;
            }
        }else{
            if(it->first != Yours_Sell_Path_1){
                continue;
            }
        }

		//查找最近的点
        for(auto itt = it->second.begin(); itt != it->second.end(); itt++){
            YoursNavPoint p = *itt;
            double distance = sqrt(pow((p.point.x - robotPose.pose.pose.position.x),2) + pow((p.point.y - robotPose.pose.pose.position.y),2) );
            if(distance < minDist){
                minDist = distance;
                locationInVector = i;
                pathName = it->first;
                result = true;
            }
            i++;
        }
		//查找最近的线
        for(auto itt = it->second.begin(); itt != it->second.end(); itt++){
            YoursNavPoint p1 = *itt;
            YoursNavPoint p2;
            int tep_int = 0;
            if (itt == (it->second.end() - 1)) {
                p2 = *it->second.begin();
                tep_int = 0;
            } else {
                p2 = *(itt + 1);
                tep_int  = (ii + 1);
            }
            rPoint rp1(p1.point.x, p1.point.y);
            rPoint rp2(p2.point.x, p2.point.y);
            rLine _line(rp1, rp2);
            double point_to_line_distance = _line.distanceToPoint(rPoint(robotPose.pose.pose.position.x,
                                         robotPose.pose.pose.position.y));
			if(point_to_line_distance < minLineToPointDist){
                minLineToPointDist = point_to_line_distance;
				yours_line.first.second = p1;
                yours_line.second.second = p2;
                yours_line.first.first = ii;
                yours_line.second.first = tep_int;
                pathName = it->first;
                result = true;
            }
            ii++;
        }
        YoursRobotPoint robot_point_tmp(robotPose.pose.pose.position.x,
                                        robotPose.pose.pose.position.y, 0);
        if (yours_line.first.second.point.distance(robot_point_tmp) > yours_line.second.second.point.distance(robot_point_tmp) ){
            locationInVector = yours_line.second.first;
        }else{
            locationInVector = yours_line.first.first;
		}
    }
   return result;
}

void followpath::navPointToRosPose(const YoursNavPoint& nP, geometry_msgs::Pose& rP){
    rP.position.x = nP.point.x;
    rP.position.y = nP.point.y;
    tf::Quaternion q = tf::createQuaternionFromYaw(nP.point.yaw);
    rP.orientation.w = q.w();
    rP.orientation.x = q.x();
    rP.orientation.y = q.y();
    rP.orientation.z = q.z();
    max_linear_speed = nP.speed;
    mRunParam.speed = nP.speed;
    mRunParam.stopWight = nP.stopWidth;
    mRunParam.stopHeight = nP.stopHeight;
	//ROS_INFO_STREAM("speed: "<<mRunParam.speed << " w: " << mRunParam.stopWight<< " h: " << mRunParam.stopHeight );
}

double distanceNavPointToRosPose(const YoursNavPoint& nP, const geometry_msgs::Pose& rP){
    
    cv::Point2d p1(nP.point.x, nP.point.y);
    cv::Point2d p2(rP.position.x, rP.position.y);
    
    return getDeltLength(p1, p2);
    
}

void followpath::trySwitchSellToChargePath(geometry_msgs::Pose &goalPose, const bool& isReplenish_){
    ROS_INFO("!!!trySwitchSellToChargePath!!!");
	double dis = distanceNavPointToRosPose(mPointMap.at(Yours_Sell_To_Charge_Path)[0], goalPose );
    if(dis < 1.0){
        mCurrentPoint.first = Yours_Sell_To_Charge_Path;
        mCurrentPoint.second = 0;
        YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
        navPointToRosPose(p, goalPose);
        std_msgs::Bool boolMsg;
        boolMsg.data = true;
        mOneloopPub.publish(boolMsg);
        if(isReplenish_){
            mRunState = FPState::RunFromSellToReplenishPath;
        }
        else{
            mRunState = FPState::RunFromSellToChargePath;
        }
                
    }else{
        switchCurrentPathNextGoal(goalPose);
    }
}

void followpath::trySwitchSellToReplenishPath(geometry_msgs::Pose &goalPose){
    //有补货路径的话执行切换到补货路径的逻辑
    if(mPointMap.count(Yours_Sell_To_Replenish_Path) > 0){
        double dis = distanceNavPointToRosPose(mPointMap.at(Yours_Sell_To_Replenish_Path)[0], goalPose );
        if(dis < 1.0){
            mCurrentPoint.first = Yours_Sell_To_Replenish_Path;
            mCurrentPoint.second = 0;
            YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
            navPointToRosPose(p, goalPose);
            std_msgs::Bool boolMsg;
            boolMsg.data = true;
            mOneloopPub.publish(boolMsg);
            mRunState = FPState::RunFromSellToReplenishPath;
        }else{
            switchCurrentPathNextGoal(goalPose);
        }
    }else{
        //没有补货路径的时候将充电路径当成是补货路径，此时到达充电点不能进行充电，而是进行补货
        trySwitchSellToChargePath(goalPose, true);
    }
}

bool isPoseEqual(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    if( fabs(p1.position.x - p2.position.x) > 0.1 ){
        return false;
    }
    
    if( fabs(p1.position.y - p2.position.y) > 0.1 ){
        return false;
    }
    
    return true;
    
}


bool followpath::robotFollowPathAmcl(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork, bool isToCharging, bool isToReplenish
    ,bool& isEnableLose 
    ,int& isEnableHaveObstaclesStopRobot
    ,bool& isReDock
    ,bool& isInitStatus
    ,bool& isRunTestPath
    ,int& cmdData
    ,int& runStatus
    ,FPState::FollowPathState &currentState
    )
{

    static int isChargingCnt = 0;
    static bool isFromBuhuo = false;
    //autodockCtrlData == 4, 关闭自动回充
    autodockCtrlData.data = 3;

    isEnableHaveObstaclesStopRobot = 1;
    
    static std_msgs::Int32 pubState;
    //pubState.data = mPowerUpPunStatus;
    bool result = false;
    static geometry_msgs::Pose goalPose;
    static geometry_msgs::Pose oldGoalPose;
    static FPState::FollowPathState oldStatus = mRunState;
    static int autoDockFailCnt=0;
    static int reDockCnt = 0;
    static int autoDockTimeOut = 0;


    if(!isPoseEqual(goalPose, oldGoalPose)){
        //ROS_INFO_STREAM("mRunState " << mRunState << " goal " << goalPose);
    }
    
    if(oldStatus != mRunState){
        ROS_INFO_STREAM("mRunState " << mRunState );
        ROS_INFO_STREAM("goal " << goalPose);
    }
    oldGoalPose = goalPose;
    oldStatus = mRunState;

    ///Avoid obstacle use data
    static int avoidGoalNumber = 0;
/*
        Init = 0,
        CalculatePath,
        RunPath,
        CheckPath,
        Calculate2edPath,
        CalculateReturnPath
        */

    if (mIsAvoidEnable)
    {
        switch (mAvoidState)
        {
        case FPState::Init:
        { /* constant-expression */
            /* code */
            avoidGoalNumber = 0; 
            break;
        }
        case FPState::CalculatePath:
        {

            break;
        }
        case FPState::RunPath:
        {
            break;
        }
        case FPState::CheckPath:
        {
            break;
        }
        case FPState::Calculate2edPath:
        {
            break;
        }
        case FPState::CalculateReturnPath:
        {
            break;
        }

        default:
            break;
        }
    }
    else
    {
        if (isInitStatus)
        {
            isInitStatus = false;
            mRunState = FPState::RunSellPath;

            mCurrentPoint.first = Yours_Sell_Path_1;
            mCurrentPoint.second = 0;
            YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
            navPointToRosPose(p, goalPose);
        }
        else if (isRunTestPath)
        {

            isRunTestPath = false;
            mRunState = FPState::RunTest1Path;
            mCurrentPoint.first = Yours_Sell_Path_2;
            mCurrentPoint.second = 0;
            YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
            navPointToRosPose(p, goalPose);
        }

        //控制数据位，用于测试
        if (cmdData == 1)
        {
            //直接执行售卖路径1的最后两个点，用于测试最终结束的路径控制
            mCurrentPoint.first = Yours_Sell_Path_1;

            if (mPointMap.at(mCurrentPoint.first).size() > 0)
            {

                if (mPointMap.at(mCurrentPoint.first).size() > 1)
                {
                    mCurrentPoint.second = (mPointMap.at(mCurrentPoint.first).size() - 2);
                }
                else
                {
                    mCurrentPoint.second = (mPointMap.at(mCurrentPoint.first).size() - 1);
                }
                YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                navPointToRosPose(p, goalPose);
                mRunState = FPState::RunSellPath;
                isStartOrStopWork = true;
            }
            cmdData = 0;
        }

        switch (mRunState)
        {
        case FPState::InitPath:
        {
            pubState.data = mPowerUpPunStatus;
            result = true;
            if (!mPointMap.empty())
            {
                if (mPointMap.count(Yours_Sell_Path_1) > 0)
                {
                    mRunState = FPState::WaitRunCommand;
                }
                else
                {
                    //path error
                }
            }
            out_put_twist = stop();
            break;
        }
        case FPState::WaitRunCommand:
        {
            result = true;

            if (isReDock)
            {
                mRunState = FPState::PowerUpReDock;
                break;
            }

            if (pubState.data == 220)
            {
                if (isToCharging)
                {
                    if ((mBattry.current > 0) || (mBattry.current == 0))
                    {
                        pubState.data = 110;
                        isStartOrStopWork = false;
                    }
                }
            }

            if (isStartOrStopWork && ( !isToCharging && !isToReplenish ) )
            {
                //mPowerUpPunStatus = 0;
                if (findNearestNavPose(mCurrentPoint.first, mCurrentPoint.second, true))
                {
                    pubState.data = 331;
                    if (mCurrentPoint.first == Yours_Sell_Path_1)
                    {
                        mRunState = FPState::RunSellPath;
                    }
                    else if (mCurrentPoint.first == Yours_Charge_To_Sell_Path)
                    {
                        mRunState = FPState::RunFromChargeToSellPath;
                    }
                    else if (mCurrentPoint.first == Yours_Replenish_To_Sell_Path)
                    {
                        mRunState = FPState::RunFromReplenishToSellPath;
                    }
                    ROS_INFO_STREAM("path " << mCurrentPoint.first << " point " << mCurrentPoint.second);
                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                    navPointToRosPose(p, goalPose);
                    //isToCharging = false;
                    //isToReplenish = false;
                }
                else
                {
                    mRunState = FPState::ErrorStopRobot;
                }
            }

            break;
        }
        case FPState::PowerUpReDock:
        {
            if (mPointMap.count(Yours_Charge_To_Sell_Path) > 0)
            {
                /*
                if (isFromBuhuo)
                {
                    pubState.data = 351;
                }
                else
                {
                    pubState.data = 341;
                }
                */
                pubState.data = 141;

                std::vector<YoursNavPoint> points = mPointMap.at(Yours_Charge_To_Sell_Path);
                YoursNavPoint goalNavPoint = points[0];
                //goalNavPoint.point.yaw = goalNavPoint.point.yaw + getRadianFormAngle(180.0);
                navPointToRosPose(goalNavPoint, goalPose);
                ROS_INFO_STREAM(" PowerUpReDock start pose << " << goalPose);
                reDockCnt++;
                autoDockTimeOut = 0;
                isStartOrStopWork = true;
                mRunState = FPState::ReturnAutoDockStartPoint;
            }
            else
            {
                mRunState = FPState::WaitRunCommand;
            }
            isReDock = false;
            result = true;
            break;
        }

        case FPState::RunSellPath:
        {
            pubState.data = 331;
            if ((isToCharging || isToReplenish ) && (mPointMap.count(Yours_Sell_To_Charge_Path) > 0))
            {
                ROS_INFO_STREAM("will CHARGIN OR REPLENISH");
                pubState.data = 341;
                if (isToReplenish)
                {
                    pubState.data = 351;
                }

                nav_msgs::Odometry robotPose = odom::getAmclPose();
                double rX = robotPose.pose.pose.position.x;
                double rY = robotPose.pose.pose.position.y;
                double rYaw = tf::getYaw(tf::Quaternion(robotPose.pose.pose.orientation.x, robotPose.pose.pose.orientation.y, robotPose.pose.pose.orientation.z, robotPose.pose.pose.orientation.w));
                YoursNavPoint robotNavPoint(rX, rY, rYaw);

                if (isToReplenish)
                {
                    if (mPointMap.count(Yours_Sell_To_Replenish_Path) > 0)
                    {
                        ExecuteRobotReturn::getRobotSellPathReturnPath(mPointMap, mCurrentPoint, robotNavPoint, false, mReturnPath, mIsForwardReturn);
                    }
                    else
                    {
                    }
                    ExecuteRobotReturn::getRobotSellPathReturnPath(mPointMap, mCurrentPoint, robotNavPoint, true, mReturnPath, mIsForwardReturn);
                    mIsReturnToCharge = false;
                }
                else
                {
                    ExecuteRobotReturn::getRobotSellPathReturnPath(mPointMap, mCurrentPoint, robotNavPoint, true, mReturnPath, mIsForwardReturn);
                    mIsReturnToCharge = true;
                }
                result = true;
                mRunState = FPState::RunPrepareSellReturnPath;
                out_put_twist = stop();
                break;
            }

            bool isGetGoal = false;
            result = runProcessing(out_put_twist, isGetGoal, goalPose);

            if (isGetGoal)
            {
                //
                if (mCurrentPoint.first == Yours_Sell_Path_1)
                {
                    if (mPointMap.count(Yours_Sell_To_Charge_Path) > 0)
                    {
                        if (isToCharging)
                        {
                            trySwitchSellToChargePath(goalPose);
                            break;
                        }
                        if (isToReplenish)
                        {
                            trySwitchSellToReplenishPath(goalPose);
                            break;
                        }

                        if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                        {
                            //suiji
                            //int r = (rand() % (int(mRandomMissionArray.size())));
                            static int loopRandomMission = 0;
                            loopRandomMission++;
                            if (loopRandomMission < mRandomMissionArray.size())
                            {
                            }
                            else
                            {
                                loopRandomMission = 0;
                            }
                            int r = loopRandomMission;

                            ROS_INFO(" random = %d   size = %d ", mRandomMissionArray[r], mRandomMissionArray.size());

                            mCurrentPoint.first = YoursPath(mRandomMissionArray[r]);
                            mCurrentPoint.second = 0;
                            //YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                            //navPointToRosPose(p, goalPose);

                            if (mIsSellPathLoop)
                            {
                                ROS_INFO("have loop");
                                YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                                navPointToRosPose(p, goalPose);
                            }
                            else
                            {
                                ROS_INFO_STREAM("not have loop  " << mCmdPathEnd.size());
                                mCurrentPoint.second = 0;
                                isStartOrStopWork = false;
                                //在路径1上，判断结束cmd
                                if (mCmdPathEnd.size() > 0)
                                {
                                    mRunState = FPState::RunEndCmdPath;
                                    isStartOrStopWork = true;
                                    mCurrentCmdPathCnt = 0;
                                }
                                else
                                {
                                    FPState::InitPath;
                                }
                            }

                            std_msgs::Bool boolMsg;
                            boolMsg.data = true;
                            mOneloopPub.publish(boolMsg);
                        }
                        else
                        {
                            switchCurrentPathNextGoal(goalPose);
                        }
                    }
                    else
                    {
                        //任务列表中没有充电任务
                        //判断是否在最后一点
                        ROS_INFO("no charge mission!!!!!!!!!!");
                        if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                        {
                            //在最后一个点
                            ROS_INFO("will random path");
                            //suiji
                            int r = rand() % int(mRandomMissionArray.size());
                            ROS_INFO("sell 1 no sell random = %d   size = %d ", YoursPath(mRandomMissionArray[r]), mRandomMissionArray.size());
                            mCurrentPoint.first = YoursPath(mRandomMissionArray[r]);
                            mCurrentPoint.second = 0;
                            if (mIsSellPathLoop)
                            {
                                ROS_INFO("have loop");
                                YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                                navPointToRosPose(p, goalPose);
                            }
                            else
                            {
                                ROS_INFO_STREAM("not have loop  " << mCmdPathEnd.size());
                                mCurrentPoint.second = 0;
                                isStartOrStopWork = false;
                                //在路径1上，判断结束cmd
                                if (mCmdPathEnd.size() > 0)
                                {
                                    mRunState = FPState::RunEndCmdPath;
                                    isStartOrStopWork = true;
                                    mCurrentCmdPathCnt = 0;
                                }
                                else
                                {
                                    FPState::InitPath;
                                }
                            }
                            //isStartOrStopWork = false;
                        }
                        else
                        {
                            //不在最后一个点，切换下一个任务
                            ROS_INFO("not last point  switch to next");
                            switchCurrentPathNextGoal(goalPose);
                        }
                    }
                }
                else
                { // not sell path 1
                    //2020.9.2 演示用路径
                    //if (mCurrentPoint.first == Yours_Sell_Path_2)
                    //{
                    //}
                    //else
                    {

                        //不是售卖任务1
                        //判断是否是最后一个点
                        if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                        {
                            //在最后一个点
                            //判断任务列表中是否有充电任务
                            if (mPointMap.count(Yours_Sell_Path_1) > 0)
                            {
                                //有充电任务
                                if (isToCharging || isToReplenish)
                                {
                                    //当前需要充电，切换任务
                                    mCurrentPoint.first = Yours_Sell_Path_1;
                                    mCurrentPoint.second = 0;
                                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                                    navPointToRosPose(p, goalPose);
                                }
                                else
                                {
                                    //不需要充电，随机选一个任务
                                    //suiji
                                    int r = rand() % int(mRandomMissionArray.size());
                                    ROS_INFO(" random = %d   size = %d ", mRandomMissionArray[r], mRandomMissionArray.size());
                                    mCurrentPoint.first = YoursPath(mRandomMissionArray[r]);
                                    mCurrentPoint.second = 0;
                                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                                    navPointToRosPose(p, goalPose);
                                }
                            }
                            else
                            {
                                //任务列表中没有充电任务
                                //随机选一个任务
                                int r = rand() % int(mRandomMissionArray.size());
                                ROS_INFO(" not 1 no sell  random = %d   size = %d ", mRandomMissionArray[r], mRandomMissionArray.size());
                                mCurrentPoint.first = YoursPath(mRandomMissionArray[r]);
                                mCurrentPoint.second = 0;
                                YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                                navPointToRosPose(p, goalPose);
                            }
                        }
                        else
                        {
                            ROS_INFO("will sw next");
                            switchCurrentPathNextGoal(goalPose);
                        }
                    }
                }
            }
            break;
        }

        case FPState::RunFromSellToChargePath:
        {
            pubState.data = 341;
            //这里只有可能是收到充电命令然后跳入的
            //jiangbo 10.20 修改，不再设置设两个值，因为要用于判断小车是否需要切换状态
            //isToCharging = false;
            //isToReplenish = false;
            //从充电状态能切换为补货或者充电么？应该是可以的。
            if (!isToCharging)
            {
                if (isToReplenish)
                {
                    pubState.data = 351;
                }
                else
                {
                    pubState.data = 331;
                }
            }

            bool isGetGoal = false;
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            if (isGetGoal)
            {
                //最后一个点
                if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                {
                    ROS_INFO("I'm at SellToChargePath last Point");
                    if(isToCharging){
                        mRunState = FPState::RunPrepareCharge;
                    }else{
                        if(isToReplenish){
                            //从充电状态改变为补货状态。
                            if(mPointMap.count(Yours_Replenish_To_Sell_Path) > 0){
                                //有独立的补货路径
                                //需要先返回售卖路径，才能转到补货路径
                                mCurrentPoint.first = Yours_Charge_To_Sell_Path;
                                mCurrentPoint.second = 0;
                                mRunState = FPState::RunFromChargeToSellPath; 
                            }else{
                                //没有独立的补货路径
                                mRunState = FPState::RunPrepareReplenish;
                            }
                        }
                        else
                        {
                            mCurrentPoint.first = Yours_Charge_To_Sell_Path;
                            mCurrentPoint.second = 0;
                            mRunState = FPState::RunFromChargeToSellPath;
                        }
                    }
                    //没到最后一个点
                }
                else
                {
                    ROS_INFO("I'm at RunFromSellToChargePath switch next goal");
                    switchCurrentPathNextGoal(goalPose);
                }
            }
            break;
        }
        case FPState::RunFromChargeToSellPath:
        {
            pubState.data = 331;
            bool isGetGoal = false;
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            if (isToCharging)
            {
                pubState.data = 341;
            }
            else if (isToReplenish)
            {
                pubState.data = 351;
            }
            if (isToReplenish || isToCharging)
            {
                if (mPointMap.count(Yours_Sell_To_Replenish_Path) <= 0)
                {
                    nav_msgs::Odometry robotOdom = odom::getAmclPose();
                    YoursNavPoint robotNavPose(robotOdom);
                    if (ExecuteRobotReturn::getRobotToSellPathReturnPath(mPointMap, robotNavPose, mCurrentPoint))
                    {
                        ROS_INFO_STREAM(" mCurrentPoint.first  mCurrentPoint.second " << mCurrentPoint.first << "  " << mCurrentPoint.second);
                        YoursNavPoint goalNavPoint = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                        navPointToRosPose(goalNavPoint, goalPose);
                        if (isToCharging)
                        {
                            mRunState = FPState::RunFromSellToChargePath;
                            break;
                        }

                        if (isToReplenish)
                        {
                            mRunState = FPState::RunFromSellToReplenishPath;
                            break;
                        }
                    }
                }
            }

            if (isGetGoal)
            {
                //最后一个点
                if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                {
                    int r = (rand() % (int(mRandomMissionArray.size())));
                    ROS_INFO(" random = %d   size = %d ", mRandomMissionArray[r], mRandomMissionArray.size());
                    //修改bug，增强适应性。让充电点可以返回任意一个靠近的售卖点
                    findNearestNavPose(mCurrentPoint.first, mCurrentPoint.second, false);
                    //mCurrentPoint.first = YoursPath(mRandomMissionArray[r]);
                    //mCurrentPoint.second = 0;

                    mRunState = FPState::RunSellPath;
                    //没到最后一个点
                }
                else
                {
                    ROS_INFO("I'm at RunFromSellToChargePath switch next goal");
                    switchCurrentPathNextGoal(goalPose);
                }
            }
            break;
        }
        case FPState::RunFromSellToReplenishPath:
        {
            pubState.data = 351;
            //这里只有从售卖路径跳入这里
            bool isGetGoal = false;
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            if(isToCharging){
                pubState.data = 341;
            }else{
                if(!isToReplenish){
                    pubState.data = 331;
                }
            }

            if (isGetGoal)
            {
                //最后一个点
                if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                {
                    if(isToCharging){
                        mRunState = FPState::RunPrepareCharge;

                    }
                    else if (isToReplenish)
                    {
                        //isToReplenish = false;
                        mRunState = FPState::RunPrepareReplenish;
                    }
                    else
                    {


                        if (mPointMap.count(Yours_Replenish_To_Sell_Path) > 0)
                        {
                            mCurrentPoint.first = Yours_Replenish_To_Sell_Path;
                        }
                        else
                        {
                            mCurrentPoint.first = Yours_Charge_To_Sell_Path;
                        }
                        mCurrentPoint.second = 0;
                        mRunState = FPState::RunFromReplenishToSellPath;

                        //for first point
                        YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                        navPointToRosPose(p, goalPose);
                    }
                }
                else
                {
                    //没到最后一个点
                    ROS_INFO("I'm at RunFromSellToReplenishPath switch next goal");
                    switchCurrentPathNextGoal(goalPose);
                }
            }
            break;
        }
        case FPState::RunFromReplenishToSellPath:
        {
            pubState.data = 331;
            if (isToCharging)
            {
                pubState.data = 341;
            }
            else if (isToReplenish)
            {
                pubState.data = 351;
            }

            bool isGetGoal = false;
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            if (isGetGoal)
            {
                //最后一个点
                if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                {
                    int r = (rand() % (int(mRandomMissionArray.size())));
                    ROS_INFO(" random = %d   size = %d ", mRandomMissionArray[r], mRandomMissionArray.size());
                    //mCurrentPoint.first = YoursPath(mRandomMissionArray[r]);
                    //mCurrentPoint.second = 0;
                    findNearestNavPose(mCurrentPoint.first, mCurrentPoint.second, false);
                    mRunState = FPState::RunSellPath;
                }
                else
                {
                    //没到最后一个点
                    ROS_INFO("I'm at RunFromReplenishToSellPath switch next goal");
                    switchCurrentPathNextGoal(goalPose);
                }
            }
            break;
        }
        case FPState::RunPrepareCharge:
        {
            pubState.data = 341;
            result = true;
            isEnableHaveObstaclesStopRobot = -1;

            //ROS_INFO("IN RunPrepareCharge");

            //if (turnAngleByWheelOdom(-90, out_put_twist))
            if (turnAngleByWheelOdom(mPreChargeAngle, out_put_twist))
            {
                //isStartOrStopWork = false;
                if (isHaveAutoCharge)
                {
                    mRunState = FPState::AutoDock;
                }
                else
                {
            	    isStartOrStopWork = false;
                    mRunState = FPState::Charging;
                }
            }
            //ROS_INFO_STREAM(out_put_twist);

            break;
        }
        case FPState::RunPrepareReplenish:
        {
            pubState.data = 351;
            result = true;
            isEnableHaveObstaclesStopRobot = -1;
            //ROS_INFO("IN RunPrepareReplenish");

            if (turnAngleByWheelOdom(mPreBuhuoAngle, out_put_twist))
            {
                //isStartOrStopWork = false;
                if (isHaveBuhuoAutoCharge)
                {
                    isFromBuhuo = true;
                    mRunState = FPState::AutoDock;
                }
                else
                {
            	    isStartOrStopWork = false;
                    mRunState = FPState::Replenish;
                }
            }

            break;
        }
        case FPState::AutoDock:
        {
            autoDockTimeOut++;
            static int kidsIrZeroCnt = 0;

            ROS_INFO_STREAM("auto dock time out " << autoDockTimeOut);
            ROS_INFO_STREAM("re dock cnt " << reDockCnt);

            result = true;

	    if(isToReplenish){
	    	isFromBuhuo = true;
	    }

	    if(isToCharging){
	    	isFromBuhuo = false;
	    }


            if (isFromBuhuo)
            {
                pubState.data = 351;
            }
            else
            {
                pubState.data = 341;
            }
            autodockCtrlData.data = 1;
            if (mDockstatus.data == 2)
            {
                isStartOrStopWork = false;
                autoDockFailCnt = 0;
                reDockCnt = 0;
                autoDockTimeOut = 0;
                if (isFromBuhuo)
                {
                    isFromBuhuo = false;
                    mRunState = FPState::Replenish;
                }
                else
                {
                    mRunState = FPState::Charging;
                }
            }
            else if (mDockstatus.data == 3)
            {
                ///autodock fail
                autoDockFailCnt++;
                //充电点
                //std::vector<YoursNavPoint> points = mPointMap.at(Yours_Sell_To_Charge_Path);
                //YoursNavPoint goalNavPoint = points[points.size() - 1];
                //goalNavPoint.point.yaw = goalNavPoint.point.yaw + getRadianFormAngle(180.0);
                std::vector<YoursNavPoint> points = mPointMap.at(Yours_Charge_To_Sell_Path);
                YoursNavPoint goalNavPoint = points[0];
                //goalNavPoint.point.yaw = goalNavPoint.point.yaw + getRadianFormAngle(180.0);
                navPointToRosPose(goalNavPoint, goalPose);
                reDockCnt++;
                autoDockTimeOut = 0;
                isStartOrStopWork = true;
                mRunState = FPState::ReturnAutoDockStartPoint;
                ROS_INFO_STREAM(" re auto dock " << reDockCnt);
            }

            //4 次重试失败
            //3 分钟
            if ((reDockCnt > 3) || (autoDockTimeOut > (10 * 60 * 3)))
            {
                autodockCtrlData.data = 4;
                std_msgs::Int32 errorCode;
                errorCode.data = 1;
                if (autoDockTimeOut > (10 * 60 * 3))
                {
                    errorCode.data = 2;
                }

                mAutoDockErrorPub.publish(errorCode);

                isStartOrStopWork = false;
                autoDockFailCnt = 0;
                reDockCnt = 0;
                autoDockTimeOut = 0;
                kidsIrZeroCnt = 0;
                if (isFromBuhuo)
                {
                    isFromBuhuo = false;
                    mRunState = FPState::Replenish;
                }
                else
                {
                    mRunState = FPState::Charging;
                }
            }

            /*
            if ((mKids.ir[0] == 0) && (mKids.ir[1] == 0) && (mKids.ir[2] == 0))
            {
                kidsIrZeroCnt++;
            }
            else
            {
                kidsIrZeroCnt = 0;
                mKids.ir[0] = 0;
                mKids.ir[1] = 0;
                mKids.ir[2] = 0;
            }
            */

            if (kidsIrZeroCnt > 10 * 30)
            {
                autodockCtrlData.data = 4;
                std_msgs::Int32 errorCode;
                errorCode.data = 3;
                mAutoDockErrorPub.publish(errorCode);
                isStartOrStopWork = false;
                autoDockFailCnt = 0;
                reDockCnt = 0;
                autoDockTimeOut = 0;
                if (isFromBuhuo)
                {
                    isFromBuhuo = false;
                    mRunState = FPState::Replenish;
                }
                else
                {
                    mRunState = FPState::Charging;
                }
                kidsIrZeroCnt = 0;
            }

            break;
        }
        case FPState::ReturnAutoDockStartPoint:
        {
            static int checkCnt = 0;
            checkCnt++;
            ROS_INFO_STREAM("check cnt " << checkCnt);
            bool isGetGoal = false;

            if (isToReplenish)
            {
                pubState.data = 351;
            }

            if (isToCharging)
            {
                pubState.data = 341;
            }

        /*
            if (isFromBuhuo)
            {
                pubState.data = 351;
            }
            else
            {
                pubState.data = 341;
            }
	    */
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            /*
            if(!result){
                ROS_INFO_STREAM("!!!!!!! NO TURN !!!!!!!");
                result = true;
            }
            */
            if (isGetGoal)
            {
                checkCnt = 0;
                //isStartOrStopWork = false;
                mRunState = FPState::AutoDock;
            }

            break;
        }

        case FPState::Charging:
        {
            static int chargeTestCnt = 0;
            chargeTestCnt++;
            result = true;
            pubState.data = 111;
            ROS_INFO_STREAM("IN Charge " << chargeTestCnt);
            isEnableHaveObstaclesStopRobot = -1;

            if (isReDock)
            {
                mRunState = FPState::PowerUpReDock;
                break;
            }

            //电池电量检测，电量大于0或者等于0。电量等于0是为了消除电量报告不准确
            if ((mBattry.current > 0) || (mBattry.current == 0))
            {
                isChargingCnt++;
            }
            else
            {
                isChargingCnt--;
            }

            //增加充电状态认证逻辑，保证到达充电位置，且在充电状态才开盖
            if (isChargingCnt > 50)
            {
                if (!isDockTest)
                {
                    pubState.data = 110;
                }
                isChargingCnt = 50;
            }

            if (chargeTestCnt > 50)
            {
                chargeTestCnt = 0;
                if (isDockTest)
                {
                    isStartOrStopWork = true;
                }
            }

            if (isStartOrStopWork)
            {
                if (isToCharging)
                {
                }
                else if (isToReplenish)
                {
                    if (mPointMap.count(Yours_Replenish_To_Sell_Path) > 0)
                    {
                        isChargingCnt = 0;
                        mCurrentPoint.first = Yours_Charge_To_Sell_Path;
                        mCurrentPoint.second = 0;
                        mRunState = FPState::RunFromChargeToSellPath;
                        //for first point
                        YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                        navPointToRosPose(p, goalPose);
                    }
                }
                else
                {
                    isChargingCnt = 0;
                    mCurrentPoint.first = Yours_Charge_To_Sell_Path;
                    mCurrentPoint.second = 0;
                    mRunState = FPState::RunFromChargeToSellPath;
                    //for first point
                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                    navPointToRosPose(p, goalPose);
                }
            }

            break;
        }
        case FPState::Replenish:
        {
            result = true;
            pubState.data = 220;
            isEnableHaveObstaclesStopRobot = -1;

            if (isToCharging)
            {
                if ((mBattry.current > 0) || (mBattry.current == 0))
                {
                    pubState.data = 110;
                }
            }

            if (mPointMap.count(Yours_Replenish_To_Sell_Path) <= 0)
            {
                if (isReDock)
                {
                    isFromBuhuo = true;
                    mRunState = FPState::PowerUpReDock;
                    break;
                }
            }

            if (isStartOrStopWork)
            {
                if (isToReplenish)
                {
                }
                else if (isToCharging)
                {
                    if (mPointMap.count(Yours_Replenish_To_Sell_Path) > 0)
                    {
                        mCurrentPoint.first = Yours_Replenish_To_Sell_Path;
                        mCurrentPoint.second = 0;
                        mRunState = FPState::RunFromReplenishToSellPath;
                        //for first point
                        YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                        navPointToRosPose(p, goalPose);
                    }
                }
                else
                {
                    if (mPointMap.count(Yours_Replenish_To_Sell_Path) > 0)
                    {
                        mCurrentPoint.first = Yours_Replenish_To_Sell_Path;
                    }
                    else
                    {
                        mCurrentPoint.first = Yours_Charge_To_Sell_Path;
                    }
                    mCurrentPoint.second = 0;
                    mRunState = FPState::RunFromReplenishToSellPath;
                    //for first point
                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                    navPointToRosPose(p, goalPose);
                }
            }

            break;
        }
        case FPState::ErrorStopRobot:
        {
            ROS_INFO("eeeeeeeeerrrrrrrrrrrrrroooooooooooorrrrrrrrrr");
            out_put_twist = stop();
            break;
        }
        case FPState::SwitchNextGoal:
        {
            break;
        }
        //根据命令路径1行走
        case FPState::RunEndCmdPath:
        {
            pubState.data = 331;
            static bool isGetGoal = false;
            result = true;
            //ROS_INFO("RunEndCmdPath");
            if (mCmdPathEnd.size() > 0)
            {
                int type = mCmdPathEnd[mCurrentCmdPathCnt].first;
                float value = mCmdPathEnd[mCurrentCmdPathCnt].second;
                if (type == 1)
                {
                    isGetGoal = this->runByWheelOdom(value, out_put_twist);
                }
                else if (type == 2)
                {
                    isGetGoal = this->turnAngleByWheelOdom(value, out_put_twist);
                }
                //到达每个目标
                if (isGetGoal)
                {
                    //最后一个点
                    if (mCurrentCmdPathCnt == (mCmdPathEnd.size() - 1))
                    {
                        isStartOrStopWork = false;
                        mRunState = FPState::InitPath;
                    }
                    else
                    {
                        mCurrentCmdPathCnt++;
                    }
                }
            }
            else
            {
                ROS_ERROR_STREAM("not have END CMD PATH");
                mRunState = FPState::InitPath;
                isStartOrStopWork = false;
            }
            break;
        }
        case FPState::RunTest1Path:
        {
            //保证当前路线在路径2上
            mCurrentPoint.first = Yours_Sell_Path_2;

            pubState.data = 331;
            bool isGetGoal = false;
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            if (isGetGoal)
            {
                //最后一个点, 切换到开始cmd路径，用于爬坡
                if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
                {
                    //没有 开始cmd路径，我认为可以直接开始走售卖路径1，因为开始不用定点走了
                    if (mCmdPathStart.size() == 0)
                    {
                    }
                    else
                    {
                        mRunState = FPState::RunStartCmdPath;
                        mCurrentCmdPathCnt = 0;
                    }
                }
                else
                {
                    switchCurrentPathNextGoal(goalPose);
                }
            }

            break;
        }
        //根据命令路径
        case FPState::RunStartCmdPath:
        {
            pubState.data = 331;
            result = true;
            if (mCmdPathStart.size() > 0)
            {
                int type = mCmdPathStart[mCurrentCmdPathCnt].first;
                float value = mCmdPathStart[mCurrentCmdPathCnt].second;
                bool isGetGoal = false;
                if (type == 1)
                {
                    isGetGoal = this->runByWheelOdom(value, out_put_twist);
                }
                else if (type == 2)
                {
                    isGetGoal = this->turnAngleByWheelOdom(value, out_put_twist);
                }
                //到达每个目标
                if (isGetGoal)
                {
                    //最后一个点
                    if (mCurrentCmdPathCnt == (mCmdPathEnd.size() - 1))
                    {
                        //isStartOrStopWork = false;
                        //mRunState = FPState::InitPath;
                        mRunState = FPState::RunSellPath;
                        mCurrentPoint.first = Yours_Sell_Path_1;
                        mCurrentPoint.second = 0;
                        //这里走完以后还要重新初始化一下定位 发布 init位置
                        geometry_msgs::PoseWithCovarianceStamped initPose;
                        initPose.header.frame_id = "map";
                        initPose.header.stamp = ros::Time::now();
                        initPose.pose.covariance[0] = 0.25;
                        initPose.pose.covariance[7] = 0.25;
                        initPose.pose.covariance[35] = 0.068;
                        initPose.pose.pose.position.x = mInitPose[0];
                        initPose.pose.pose.position.y = mInitPose[1];
                        tf::Quaternion q = tf::createQuaternionFromYaw(mInitPose[2]);
                        initPose.pose.pose.orientation.x = q.getX();
                        initPose.pose.pose.orientation.y = q.getY();
                        initPose.pose.pose.orientation.z = q.getZ();
                        initPose.pose.pose.orientation.w = q.getW();
                        mInitPosePub.publish(initPose);
                    }
                    else
                    {
                        mCurrentCmdPathCnt++;
                    }
                }
            }
            else
            {
                ROS_ERROR_STREAM("not have START CMD PATH");
                mRunState = FPState::InitPath;
                isStartOrStopWork = false;
            }
            break;
        }
        case FPState::RunPrepareSellReturnPath:
        {
            if (mIsReturnToCharge) {
                pubState.data = 341;
            } else {
                pubState.data = 351;
            }
            result = true;
            YoursNavPoint rGoal = mReturnPath[0];
            navPointToRosPose(rGoal, goalPose);
            out_put_twist = stop();
            if (!mIsForwardReturn)
            {
                isEnableHaveObstaclesStopRobot = -2;
                if (turnAngleByWheelOdom(180.0, out_put_twist))
                {
                    mRunState = FPState::RunSellReturnPath;
                }
            }
            else
            {
                mRunState = FPState::RunSellReturnPath;
            }
            break;
        }
        case FPState::RunSellReturnPath:
        {
            if (mIsReturnToCharge)
            {
                pubState.data = 341;
            }
            else
            {
                pubState.data = 351;
            }

            if (isToCharging)
            {
                pubState.data = 341;
                mIsReturnToCharge = true;
            }
            else if (isToReplenish)
            {
                pubState.data = 351;
                mIsReturnToCharge = false;
            }
            else
            {
                pubState.data = 331;
                mIsReturnToCharge = false;
            }

            bool isGetGoal = false;
            static int pathPointCnt = 0;
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            if (isGetGoal)
            {
                pathPointCnt++;
                if (pathPointCnt >= mReturnPath.size())
                {
                    //到达最后一个点，开始走返回补货，或者返回路径
                    if(pubState.data == 331){
                        //若不用充电或补货，则继续工作

                        findNearestNavPose(mCurrentPoint.first, mCurrentPoint.second, false);
                        YoursNavPoint rGoal = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                        navPointToRosPose(rGoal, goalPose);
                        mRunState = FPState::RunSellPath;
                        ROS_INFO_STREAM("hahah  bug  pathPointCnt = " << pathPointCnt);
                        pathPointCnt = 0;
                    } else {
                        if (mIsReturnToCharge)
                        {
                            YoursNavPoint rGoal = mPointMap.at(Yours_Sell_To_Charge_Path)[0];
                            navPointToRosPose(rGoal, goalPose);
                            mCurrentPoint.first = Yours_Sell_To_Charge_Path;
                            mCurrentPoint.second = 0;
                            mRunState = FPState::RunFromSellToChargePath;
                            pathPointCnt = 0;
                        }
                        else
                        {
                            if (mPointMap.count(Yours_Sell_To_Replenish_Path) > 0)
                            {
                                //有补货路线，需要判断，走到这里是补货还是充电
                                YoursNavPoint rGoal = mPointMap.at(Yours_Sell_To_Replenish_Path)[0];
                                navPointToRosPose(rGoal, goalPose);
                                mCurrentPoint.first = Yours_Sell_To_Replenish_Path;
                                mCurrentPoint.second = 0;
                            }
                            else
                            {
                                YoursNavPoint rGoal = mPointMap.at(Yours_Sell_To_Charge_Path)[0];
                                navPointToRosPose(rGoal, goalPose);
                                mCurrentPoint.first = Yours_Sell_To_Charge_Path;
                                mCurrentPoint.second = 0;
                            }
                            mRunState = FPState::RunFromSellToReplenishPath;
                            pathPointCnt = 0;
                        }
                    }
                }
                else
                {
                    YoursNavPoint rGoal = mReturnPath[pathPointCnt];
                    navPointToRosPose(rGoal, goalPose);
                }
            }
            break;
        }
        }
    }
    //int r = rand()%int(mRandomMissionArray.size());
    //ROS_INFO(" random = %d   size = %d  mission = %d", r, mRandomMissionArray.size(), mRandomMissionArray[r]); 

    if(pubState.data != 111 && pubState.data != 221){
        if(!isStartOrStopWork){
            pubState.data = pubState.data & 0xFFFE;
        }
    }

    runStatus = pubState.data;
	mPubState = pubState;
	mPubRunState.publish(pubState);
    /*
    if(pubState.data == 110){
        usleep(1000*1000);
        mPubRunState.publish(pubState);
        usleep(1000*1000);
        mPubRunState.publish(pubState);
    }
    */

    mAutodockCtrlPub.publish(autodockCtrlData);

    updataRobotToDock();

    geometry_msgs::PoseStamped pubGoal;
    pubGoal.header.frame_id = "map";
    pubGoal.header.stamp = ros::Time::now();
    pubGoal.pose = goalPose;
    mGoalPub.publish(pubGoal);

    currentState = mRunState;

    static YoursPath old_path = Yours_Path_Null;
    if(old_path != mCurrentPoint.first){
        mLog->info(" mCurrentPoint path " + std::to_string(mCurrentPoint.first));
        old_path = mCurrentPoint.first;
    }

    if(!isStartOrStopWork || isPlugIn){
        out_put_twist = stop();
        return true;
    }

    return result;
    
}

void followpath::WriteRunState(const FPState::FollowPathState& _state, std::shared_ptr<yours_log::YoursLog>& _log){
    if (_state == FPState::InitPath) {
        _log->info("Follow Path State : InitPath");
    } else if (_state == FPState::WaitRunCommand) {
        _log->info("Follow Path State : WaitRunCommand");
    } else if (_state == FPState::RunSellPath) {
        _log->info("Follow Path State : RunSellPath");
    } else if (_state == FPState::PowerUpReDock) {
        _log->info("Follow Path State : PowerUpReDock");
    } else if (_state == FPState::RunFromSellToChargePath) {
        _log->info("Follow Path State : RunFromSellToChargePath");
    } else if (_state == FPState::RunFromChargeToSellPath) {
        _log->info("Follow Path State : RunFromChargeToSellPath");
    } else if (_state == FPState::RunFromSellToReplenishPath) {
        _log->info("Follow Path State : RunFromSellToReplenishPath");
    } else if (_state == FPState::RunFromReplenishToSellPath) {
        _log->info("Follow Path State : RunFromReplenishToSellPath");
    } else if (_state == FPState::RunPrepareCharge) {
        _log->info("Follow Path State : RunPrepareCharge");
    } else if (_state == FPState::RunPrepareReplenish) {
        _log->info("Follow Path State : RunPrepareReplenish");
    } else if (_state == FPState::AutoDock) {
        _log->info("Follow Path State : AutoDock");
    } else if (_state == FPState::ReturnAutoDockStartPoint) {
        _log->info("Follow Path State : ReturnAutoDockStartPoint");
    } else if (_state == FPState::Charging) {
        _log->info("Follow Path State : Charging");
    } else if (_state == FPState::Replenish) {
        _log->info("Follow Path State : Replenish");
    } else if (_state == FPState::ErrorStopRobot) {
        _log->info("Follow Path State : ErrorStopRobot");
    } else if (_state == FPState::SwitchNextGoal) {
        _log->info("Follow Path State : SwitchNextGoal");
    } else if (_state == FPState::RunEndCmdPath) {
        _log->info("Follow Path State : RunEndCmdPath");
    } else if (_state == FPState::RunTest1Path) {
        _log->info("Follow Path State : RunTest1Path");
    } else if (_state == FPState::RunStartCmdPath) {
        _log->info("Follow Path State : RunStartCmdPath");
    } else if (_state == FPState::RunPrepareSellReturnPath) {
        _log->info("Follow Path State : RunPrepareSellReturnPath");
    } else if (_state == FPState::RunSellReturnPath) {
        _log->info("Follow Path State : RunSellReturnPath");
    } else {
        _log->info("Follow Path State : State Error ");
    }
}

bool followpath::RunRemapPath(geometry_msgs::Twist& out_put_twist, bool& isStartOrStopWork) {
	//局部变量
    static std_msgs::Int32 pubState;
    static int pubState_old = pubState.data;
    static FPState::FollowPathState mRunState_old = mRunState;
    //static geometry_msgs::Pose goalPose;
    bool result = false;
    // log写入 状态切换记录，当前返回状态记录

    //状态切换
    switch (mRunState) {
        case FPState::InitPath: {
            pubState.data = 702;
            mCurrentPoint.first = Yours_Sell_Path_1;
            mCurrentPoint.second = 0;
            if (!mPointMap.empty()) {
				//存在路径文件
                if (findNearestNavPose(mCurrentPoint.first,
                                       mCurrentPoint.second, true)) {
                    mRunState = FPState::WaitRunCommand;
                }
            }else{
				//不存在路径文件
                pubState.data = 741;
            }
            result = true;
            break;
        }
		case FPState::WaitRunCommand:{
            pubState.data = 702;
			if(isStartOrStopWork){
                YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                navPointToRosPose(p, mGoalPose);
                mRunState = FPState::RunSellPath;
            }
            
			result = true;
            break;
        }

        case FPState::RunSellPath: {
            pubState.data = 701;
			if(!isStartOrStopWork){
                pubState.data = 700;
            }
   
            bool isGetGoal = false;
            result = runProcessing(out_put_twist, isGetGoal, mGoalPose);

			if(isGetGoal){
                if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1)){						
					//走到最后一个点, 准备停车结束重新建图
                    isStartOrStopWork = false;
                    mCurrentPoint.second = 0;
                    mRunState = FPState::WaitRunCommand;
                } else {
                    //不是最后一个点，切换到下一个点
	                switchCurrentPathNextGoal(mGoalPose);
                }
            }

            break;
        }
        default: {

            break;
        }
    }

    //log
	if(pubState_old != pubState.data){
        pubState_old = pubState.data;
        mLog->info("follow_path swich pub_state to: " + std::to_string(pubState.data));
    }

	if(mRunState_old != mRunState){
        mRunState_old = mRunState;
        WriteRunState(mRunState, mLog);
    }

    //数据上报和返回
    mPubState = pubState;
	mPubRunState.publish(pubState);

    return result;
}

void followpath::updataRobotToDock(){
    nav_msgs::Odometry pose = odom::getAmclPose();
    float chargeDist = 60000.0;
    float reDist = 60000.0;
    float minDist = 9999.0;

    if(mPointMap.count(Yours_Sell_To_Charge_Path) > 0){
        YoursNavPoint sp = mPointMap.at(Yours_Sell_To_Charge_Path)[mPointMap.at(Yours_Sell_To_Charge_Path).size() - 1];
        cv::Point2d p1, p2;
        p1.x = sp.point.x;
        p1.y = sp.point.y;
        p2.x = pose.pose.pose.position.x;
        p2.y = pose.pose.pose.position.y;
        chargeDist = getDeltLength(p1, p2);
        minDist = chargeDist;
        if (mPointMap.count(Yours_Sell_To_Replenish_Path) > 0)
        {
            YoursNavPoint sp = mPointMap.at(Yours_Sell_To_Replenish_Path)[mPointMap.at(Yours_Sell_To_Replenish_Path).size() - 1];
            p1.x = sp.point.x;
            p1.y = sp.point.y;
            p2.x = pose.pose.pose.position.x;
            p2.y = pose.pose.pose.position.y;
            reDist = getDeltLength(p1, p2);
            if(reDist < minDist){
                minDist = reDist;
            }
        }
    }

    std_msgs::Float32 pubData;
    pubData.data = minDist;
    mRobotToDockPub.publish(pubData);
}


void followpath::switchCurrentPathNextGoal(geometry_msgs::Pose& goalPose)
{
    int size = mPointMap.at(mCurrentPoint.first).size();
    YoursNavPoint p;
    if(mCurrentPoint.second == (size - 1)){
        mCurrentPoint.second = 0;
    }else{
        mCurrentPoint.second++;
    }
    p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
    navPointToRosPose(p, goalPose);
}

void followpath::updatePath(const std::map< YoursPath, std::vector< YoursNavPoint > >& pointMap, const std::vector< YoursPath >& randomMissionArray)
{
    mPointMap.clear();
    mPointMap = pointMap;
    ROS_INFO("!!!! update path");
    mPathLine.clear();
    for(auto it = mPointMap.begin(); it != mPointMap.end(); it++){
        ROS_INFO_STREAM("Path name " << it->first);
        std::vector<YoursNavPoint> ps = it->second;
        for(int i = 0; i < ps.size(); i++){
            ROS_INFO_STREAM("p: " << i << " : x " << ps[i].point.x << " y " << ps[i].point.y << " yaw " << ps[i].point.yaw );
            if(i > 0){
                rPoint p1;
                rPoint p2;
                p1.x = ps[i - 1].point.x;
                p1.y = ps[i - 1].point.y;
                p2.x = ps[i].point.x;
                p2.y = ps[i].point.y;
                rLine line(p1, p2);
                mPathLine.push_back(line);
            }
        }
    }
    
    mRandomMissionArray = randomMissionArray;

    if (mPointMap.count(mCurrentPoint.first) != 0) {
        if (mPointMap.at(mCurrentPoint.first).size() > mCurrentPoint.second) {
    		YoursNavPoint p;
            p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
            navPointToRosPose(p, mGoalPose);
        }
    }
    for(int i = 0 ; i < mRandomMissionArray.size(); i ++ ){
	ROS_INFO("random %d = %d",i,mRandomMissionArray[i] );
	
	}
}


void followpath::getStopSpeedPara(float& speed, float& stopWight, float& stopHeight){
    speed = mRunParam.speed;
    stopWight = mRunParam.stopWight; 
    stopHeight = mRunParam.stopHeight; 
}


void followpath::setAvoidEnable(bool isEnable_){
    mIsAvoidEnable = isEnable_;
}

void followpath::setLidarDetectInfo(const std_msgs::Float32MultiArray &lidarInfo_)
{
    mLidarInfo = lidarInfo_;
}

/*
nav_msgs::Odometry getNextAvoidObstaclePose(const nav_msgs::Odometry inPose_, double turnAngle_, double distance_){
    cv::Point2d basePoint;
    basePoint.x = cos( getRadianFormAngle(turnAngle_) ) * distance_;
    basePoint.y = sin( getRadianFormAngle(turnAngle_) ) * distance_;
    cv::Point2d goalPoint;
    tf::Quaternion q;
    q.setX(inPose_.pose.pose.orientation.x);
    q.setY(inPose_.pose.pose.orientation.y);
    q.setZ(inPose_.pose.pose.orientation.z);
    q.setW(inPose_.pose.pose.orientation.w);
    double yaw = tf::getYaw(q);
    goalPoint.x = basePoint.x * cos(yaw) + basePoint.y * sin(yaw) + inPose_.pose.pose.position.x;
    goalPoint.y = basePoint.y * cos(yaw) - basePoint.x * sin(yaw) + inPose_.pose.pose.position.x;

    double goalYaw = yaw +  getRadianFormAngle(turnAngle_);
    tf::Quaternion goalQ = tf::createQuaternionFromYaw(goalYaw);

    nav_msgs::Odometry goalOdom;
    goalOdom.pose.pose.position.x = goalPoint.x;
    goalOdom.pose.pose.position.y = goalPoint.y;
    goalOdom.pose.pose.position.z = inPose_.pose.pose.position.z;
    goalOdom.pose.pose.orientation.x = goalQ.getX();
    goalOdom.pose.pose.orientation.y = goalQ.getY();
    goalOdom.pose.pose.orientation.z = goalQ.getZ();
    goalOdom.pose.pose.orientation.w = goalQ.getW();
    return goalOdom;
}
*/

nav_msgs::Odometry getNextAvoidObstaclePose(const nav_msgs::Odometry inPose_, double turnAngle_, double distance_){
    cv::Point2d basePoint;
    tf::Quaternion forwardQ(inPose_.pose.pose.orientation.x, inPose_.pose.pose.orientation.y, inPose_.pose.pose.orientation.z, inPose_.pose.pose.orientation.w);
    double forwardYaw = tf::getYaw(forwardQ);
    basePoint.x = cos( forwardYaw ) * distance_;
    basePoint.y = sin( forwardYaw ) * distance_;
    cv::Point2d goalPoint;
    
    goalPoint.x = basePoint.x + inPose_.pose.pose.position.x;
    goalPoint.y = basePoint.y + inPose_.pose.pose.position.y;

    double goalYaw =  forwardYaw + getRadianFormAngle(turnAngle_);  //yaw +  getRadianFormAngle(turnAngle_);
    tf::Quaternion goalQ = tf::createQuaternionFromYaw(goalYaw);

    nav_msgs::Odometry goalOdom;
    goalOdom.pose.pose.position.x = goalPoint.x;
    goalOdom.pose.pose.position.y = goalPoint.y;
    goalOdom.pose.pose.position.z = inPose_.pose.pose.position.z;
    goalOdom.pose.pose.orientation.x = goalQ.getX();
    goalOdom.pose.pose.orientation.y = goalQ.getY();
    goalOdom.pose.pose.orientation.z = goalQ.getZ();
    goalOdom.pose.pose.orientation.w = goalQ.getW();
    return goalOdom;
}

inline double getPoseYaw(const geometry_msgs::Pose& pose_){
    tf::Quaternion q(pose_.orientation.x, pose_.orientation.y,pose_.orientation.z, pose_.orientation.w);
    return tf::getYaw(q);
}

std::vector<AvoidPathCell> getAvoidObstaclePath(const nav_msgs::Odometry& currentPose, const bool& isTurnLeft, const double& turnAngle, const double& sideDistance,geometry_msgs::PoseArray& debugPoseArray){
    std::vector<AvoidPathCell> paths;
    debugPoseArray.poses.clear();
    debugPoseArray.header.frame_id = "map";

    nav_msgs::Odometry basePose;
    nav_msgs::Odometry goalPose1;
    nav_msgs::Odometry goalPose2;
    nav_msgs::Odometry goalPose3;

    tf::Quaternion currentQ(currentPose.pose.pose.orientation.x, currentPose.pose.pose.orientation.y, currentPose.pose.pose.orientation.z, currentPose.pose.pose.orientation.w );
    double currentYaw = tf::getYaw(currentQ);

    basePose.pose.pose.position = currentPose.pose.pose.position;

    tf::Quaternion baseYaw = tf::createQuaternionFromYaw(getRadianFormAngle(turnAngle) + currentYaw);//  (0,0,0,1);

    basePose.pose.pose.orientation.x = baseYaw.x();
    basePose.pose.pose.orientation.y = baseYaw.y();
    basePose.pose.pose.orientation.z = baseYaw.z();
    basePose.pose.pose.orientation.w = baseYaw.w();

    goalPose1 = getNextAvoidObstaclePose(basePose, -turnAngle, sideDistance);
    goalPose2 = getNextAvoidObstaclePose(goalPose1, -turnAngle, sideDistance);
    goalPose3 = getNextAvoidObstaclePose(goalPose2, turnAngle, sideDistance);

    geometry_msgs::Pose p0 = currentPose.pose.pose;
    geometry_msgs::Pose p1 = basePose.pose.pose;
    geometry_msgs::Pose p2 = goalPose1.pose.pose;
    geometry_msgs::Pose p3 = goalPose2.pose.pose;
    geometry_msgs::Pose p4 = goalPose3.pose.pose;

    geometry_msgs::PoseArray pa;
    pa.poses.push_back(p0);
    pa.poses.push_back(p1);
    pa.poses.push_back(p2);
    pa.poses.push_back(p3);
    pa.poses.push_back(p4);
    pa.header.frame_id = "map";
    debugPoseArray = pa;

    AvoidPathCell cell;
    cell.isTurn = true;
    cell.turnAngle = turnAngle;
    paths.push_back(cell);
    cell.isTurn = false;
    cell.goalPoint.point.x = goalPose1.pose.pose.position.x;
    cell.goalPoint.point.y = goalPose1.pose.pose.position.y;
    cell.goalPoint.point.yaw = getPoseYaw(p2);
    cell.goalPoint.speed = 0.4;
    cell.goalPoint.stopWidth = 0.6;
    cell.goalPoint.stopHeight = 1.0;
    paths.push_back(cell);
    cell.goalPoint.point.x = goalPose2.pose.pose.position.x;
    cell.goalPoint.point.y = goalPose2.pose.pose.position.y;
    cell.goalPoint.point.yaw = getPoseYaw(p3);
    paths.push_back(cell);
    cell.goalPoint.point.x = goalPose3.pose.pose.position.x;
    cell.goalPoint.point.y = goalPose3.pose.pose.position.y;
    cell.goalPoint.point.yaw = getPoseYaw(p4);
    paths.push_back(cell);
    return paths;
}

double getTurnAngle(const double& distanceFormObstacle){
    if(distanceFormObstacle < 1.0){
        return 45.0;
    }else{
        return 60.0;
    }
}

std::vector<AvoidPathCell> followpath::CalculateAvoidPath(const std_msgs::Float32MultiArray& lidarInfo_)
{
    std::vector<AvoidPathCell> path;
    nav_msgs::Odometry currentPose = odom::getAmclPose();
    bool isTurnLeft = true;
    ///走障碍物窄的一侧
    if (lidarInfo_.data[1] > fabs(lidarInfo_.data[2]))
    {
        isTurnLeft = false;
    }

    double turnAngle = getTurnAngle(lidarInfo_.data[0]);
    if (!isTurnLeft)
    {
        turnAngle = 0 - turnAngle;
    }
    double sideDistance = 1.0;
    geometry_msgs::PoseArray debugPoseArray;
    mAvoidPathVector = getAvoidObstaclePath(currentPose, isTurnLeft, turnAngle, sideDistance, debugPoseArray);
    double allDistance =  (2*sideDistance * cos(getRadianFormAngle(turnAngle)))   + sideDistance;
    


    ///开始计算第一个目标点


    return path;
}

void followpath::autodockStatusCallback(const std_msgs::Int32 &msg)
{
    mDockstatus = msg;
    if(mDockstatus.data == 2){
        ROS_INFO_STREAM(" auto dock ok ");
    }else if(mDockstatus.data == 3){
        ROS_INFO_STREAM(" auto dock error ");
    }
}

bool followpath::robotFollowPathZhiheng(geometry_msgs::Twist &out_put_twist, bool& isStartOrStopWork, bool& isCleanPath)
{
    bool result = false;
    static int poseCnt = 0;
    switch (mZhihengRunState)
    {
    case FPState::ZhihengInitPath /* constant-expression */:
    {
        /* code */
        poseCnt = 0;
        if(mPoseArray.poses.size() != 0){
            mZhihengGoal = mPoseArray.poses[0];
            mZhihengRunState = FPState::ZhihengWaitRun;
        }
        out_put_twist = stop();
        result = true;
        break;
    }
    case FPState::ZhihengWaitRun:
    {
        if(isStartOrStopWork){
            mZhihengRunState = FPState::ZhihengRun;
        }
        out_put_twist = stop();
        result = true;
        
        break;
    }
    case FPState::ZhihengRun:
    {
        bool isGetGoal = false;
        result = runProcessing(out_put_twist, isGetGoal, mZhihengGoal);
        if(isGetGoal){
            ///只有一个点，所以跑一次
            if(mPoseArray.poses.size() == 1){
                mPoseArray.poses.clear();
                isStartOrStopWork = false;
                mZhihengRunState = FPState::ZhihengInitPath;
            }else{
                poseCnt++;
                ///当前路径最后一个点
                if(poseCnt == mPoseArray.poses.size()){
                    poseCnt = 0;
                }
                mZhihengGoal = mPoseArray.poses[poseCnt];
            }
        }
        if(isCleanPath){
            isCleanPath = false;
            isStartOrStopWork = false;
            mZhihengRunState = FPState::ZhihengInitPath;
        }
        break;
    }
    default:
        break;
    }

    if(!isStartOrStopWork){
        out_put_twist = stop();
    }

    return result;
}



void followpath::zhihengUpdatePath(const geometry_msgs::PoseArray& path){
    mPoseArray = path;
}


struct  Vec2d
{
	double x, y;

	Vec2d()
	{
		x = 0.0;
		y = 0.0;
	}
	Vec2d(double dx, double dy)
	{
		x = dx;
		y = dy;
	}
	void Set(double dx, double dy)
	{
		x = dx;
		y = dy;
	}
};

bool IsPointOnLine(double px0, double py0, double px1, double py1, double px2, double py2)
{
	bool flag = false;
	double d1 = (px1 - px0) * (py2 - py0) - (px2 - px0) * (py1 - py0);
	if ((fabs(d1) < EPSILON) && ((px0 - px1) * (px0 - px2) <= 0) && ((py0 - py1) * (py0 - py2) <= 0))
	{
		flag = true;
	}
	return flag;
}

//判断两线段相交
bool IsIntersect(double px1, double py1, double px2, double py2, double px3, double py3, double px4, double py4)
{
	bool flag = false;
	double d = (px2 - px1) * (py4 - py3) - (py2 - py1) * (px4 - px3);
//	if (d != 0)

    if(fabs(d) > EPSILON)
	{
		double r = ((py1 - py3) * (px4 - px3) - (px1 - px3) * (py4 - py3)) / d;
		double s = ((py1 - py3) * (px2 - px1) - (px1 - px3) * (py2 - py1)) / d;
		if ((r >= 0.) && (r <= 1.0) && (s >= 0.) && (s <= 1.))
		{
			flag = true;
		}
	}
	return flag;
}

//判断点在多边形内
bool Point_In_Polygon_2D(double x, double y, const std::vector<rPoint> &POL)
{	
	bool isInside = false;
	int count = 0;
	
	//
	//double minX = DBL_MAX;
	double minX = 10000.0;
	for (int i = 0; i < POL.size(); i++)
	{
		//minX = std::min(minX, POL[i].x);
	}

	//
	double px = x;
	double py = y;
	double linePoint1x = x;
	double linePoint1y = y;
	double linePoint2x = minX -10;			//取最小的X值还小的值作为射线的终点
	double linePoint2y = y;
double cx1 = 0.0;
double cy1 = 0.0;
double cx2 = 0.0;
double cy2 = 0.0;
	//遍历每一条边
	for (int i = 0; i < POL.size(); i++)
	{
        if (i == (POL.size() - 1))
        {
            cx1 = POL[i].x;
            cy1 = POL[i].y;
            cx2 = POL[0].x;
            cy2 = POL[0].y;
        }
        else
        {
            cx1 = POL[i].x;
            cy1 = POL[i].y;
            cx2 = POL[i + 1].x;
            cy2 = POL[i + 1].y;
        }

        if (IsPointOnLine(px, py, cx1, cy1, cx2, cy2))
		{
            std::cout<<"!!!!! on line !!!!!" << std::endl;
			return true;
		}

		if (fabs(cy2 - cy1) < EPSILON)   //平行则不相交
		{
			continue;
		}

		if (IsPointOnLine(cx1, cy1, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
		{
			if (cy1 > cy2)			//只保证上端点+1
			{
				count++;
			}
		}
		else if (IsPointOnLine(cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
		{
			if (cy2 > cy1)			//只保证上端点+1
			{
				count++;
			}
		}
		else if (IsIntersect(cx1, cy1, cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))   //已经排除平行的情况
		{
			count++;
		}
	}
	
	if (count % 2 == 1)
	{
        std::cout << count << std::endl;
		isInside = true;
	}

	return isInside;
}

bool followpath::isRobotInThisZone(const rPoint& p, const rZone& zone){
    return Point_In_Polygon_2D(p.x, p.y, zone.points);
}

bool followpath::isRobotInDangerZones( const rPoint& p, int& zoneNumber){

    //nav_msgs::Odometry robotPose = odom::getAmclPose();
    bool flag = false;
    int zoneCnt = 0;
    for(auto i:mDangerZone){
       // i.printZone(); 
        if(isRobotInThisZone(p, i)){
            flag = true;
            zoneNumber = zoneCnt;
            break;
        }
        zoneCnt++;
    }
    return flag;
}

void  followpath::updateDangerzone(const std::vector<rZone>& zone){
    mDangerZone = zone;
}


bool followpath::isRobotOutPath(const rPoint& p){
    
    bool flag = true;
    for(auto i:mPathLine){
        double dist = i.distanceToPoint(p);
        if(dist < mPathDistance){
            ROS_INFO_STREAM("i am  " << dist);
            //return true;
            flag = false;
        }

    }
    return flag;
}

void  followpath::localPathCallBack(const nav_msgs::Path& msg){
    mIsGetLocalPath = true;
    mLocalPath = msg;
}

//用于检查当前
bool isGoalForwardRobot(const geometry_msgs::Pose& goalPose, const geometry_msgs::Pose& robotPose, double& yaw){
    YoursNavPoint p1(robotPose.position.x, robotPose.position.y, 0);
    YoursNavPoint p2(goalPose.position.x, goalPose.position.y, 0);
    yaw = p1.getYaw(p2);
    yaw = (yaw/M_PI)*180.0; //to angle

    double robotYaw = 0;
    robotYaw = tf::getYaw(tf::Quaternion(robotPose.orientation.x, robotPose.orientation.y, robotPose.orientation.z, robotPose.orientation.w));
    yaw = fabs( (robotYaw/M_PI)*180 - yaw );
    if(yaw > 180){
        yaw = yaw - 180;
    }

    if(yaw > 90){
        return false;
    }
    return true;
}

int  getForwardGoal(const nav_msgs::Path& msg, const nav_msgs::Odometry& location){
    double minDist = 999.0;
    int minCnt = 0; 
    for(int i = 0; i < msg.poses.size(); i++){
        geometry_msgs::Pose pathPose = msg.poses[i].pose;
        geometry_msgs::Pose robotPose = odom::getAmclPose().pose.pose;
        double yaw = 0.;
        if( isGoalForwardRobot(pathPose, robotPose, yaw) ){
            double dist = getDeltLength(pathPose, robotPose);
            if((minDist > dist) && (dist > 0.1)){
                minDist = dist;
                minCnt = i;
            }
        }
    }

    return minCnt;

}

void followpath::testPubGlobalPath(){
#if 0
        YoursNavPoint globalGoal1 = mPointMap.at(Yours_Sell_Path_1)[0]; //这里有角度值
        YoursNavPoint globalGoal2 = mPointMap.at(Yours_Sell_Path_1)[1]; //这里有角度值
        nav_msgs::Odometry currentPose  = odom::getOdomPose();
        nav_msgs::Path globalPathPub;
        
        geometry_msgs::PoseStamped  p1;
        
        //p1.pose = currentPose.pose.pose;
         

        p1.pose.position.x = globalGoal1.point.x;
        p1.pose.position.y = globalGoal1.point.y;
        tf::Quaternion q = tf::createQuaternionFromYaw(globalGoal1.point.yaw);
        ROS_INFO_STREAM("yaw 1"  << globalGoal1.point.yaw );
        p1.pose.orientation.x = q.getX();
        p1.pose.orientation.y = q.getY();
        p1.pose.orientation.z = q.getZ();
        p1.pose.orientation.w = q.getW();
        globalPathPub.poses.push_back(p1);
        
        p1.pose.position.x = globalGoal2.point.x;
        p1.pose.position.y = globalGoal2.point.y;
        q = tf::createQuaternionFromYaw(globalGoal2.point.yaw);
        ROS_INFO_STREAM("yaw 2"  << globalGoal2.point.yaw );
        p1.pose.orientation.x = q.getX();
        p1.pose.orientation.y = q.getY();
        p1.pose.orientation.z = q.getZ();
        p1.pose.orientation.w = q.getW();
        globalPathPub.poses.push_back(p1);
        mGlobalPathPub.publish(globalPathPub);
#endif

#if 1
        YoursNavPoint globalGoal = mPointMap.at(Yours_Sell_Path_1)[0]; //这里有角度值
        
        nav_msgs::Odometry currentPose  = odom::getAmclPose();
        nav_msgs::Path globalPathPub;
        
        geometry_msgs::PoseStamped  p1;
        p1.pose = currentPose.pose.pose;
        globalPathPub.poses.push_back(p1);
        p1.pose.position.x = globalGoal.point.x;
        p1.pose.position.y = globalGoal.point.y;
        tf::Quaternion q = tf::createQuaternionFromYaw(globalGoal.point.yaw);
        ROS_INFO_STREAM("yaw  "  << globalGoal.point.yaw );
        p1.pose.orientation.x = q.getX();
        p1.pose.orientation.y = q.getY();
        p1.pose.orientation.z = q.getZ();
        p1.pose.orientation.w = q.getW();
        globalPathPub.poses.push_back(p1);
        mGlobalPathPub.publish(globalPathPub);
#endif
}

void followpath::updataGoal(){

}


bool followpath::robotFollowPathLocalPlanTest(geometry_msgs::Twist &out_put_twist, bool &isStartOrStopWork)
{
    ROS_INFO("2");
    static std_msgs::Int32 pubState;
    //if (pubState.data == 0)
    //{
        pubState.data = mPowerUpPunStatus;
    //}
    bool result = false;
    static geometry_msgs::Pose goalPose;
    static bool isAvoidObstacle = false;

    static FPState::FollowPathState oldStatus = mRunState;
    
    
    if(oldStatus != mRunState){
        ROS_INFO_STREAM("mRunState " << mRunState );
        ROS_INFO_STREAM("goal " << goalPose);
    }
    oldStatus = mRunState;


    switch (mRunState)
    {
    case FPState::InitPath:
    {
        /* code */
        mCurrentPoint.first = Yours_Sell_Path_1;
        mCurrentPoint.second = 0;
        mRunState = FPState::WaitRunCommand;
        break;
    }
    case FPState::WaitRunCommand:
    {
        if(isStartOrStopWork){
            YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
            navPointToRosPose(p, goalPose);
            mRunState = FPState::RunSellPath;
        }

        break;
    }
    case FPState::RunSellPath:
    {
        odom::getAmclPose;
        bool isGetGoal = false;
        result = runProcessing(out_put_twist, isGetGoal, goalPose);
        if (isGetGoal)
        {
            if (mCurrentPoint.second == (mPointMap.at(mCurrentPoint.first).size() - 1))
            {
                mCurrentPoint.second = 0;
                mCurrentPoint.first = Yours_Sell_Path_1;
                YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                navPointToRosPose(p, goalPose);
            }
            else
            {
                switchCurrentPathNextGoal(goalPose);
            }
            isAvoidObstacle = true;
        }

        double ddd = getDeltLength(odom::getAmclPose().pose.pose, goalPose);
        if(ddd > 0.5){
            isAvoidObstacle = true;
        }


        if(isAvoidObstacle){
            mRunState = FPState::RunAvoidObstaclePath;
            mLocalPath.poses.clear();
        }

        break;
    }

    case FPState::RunAvoidObstaclePath:
    {
        ///发布需要计算局部路径的当前点和目标点；
        ROS_INFO("--------");
        YoursNavPoint globalGoal = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second]; //这里有角度值
        nav_msgs::Odometry currentPose  = odom::getAmclPose();
        nav_msgs::Path globalPathPub;
        
        geometry_msgs::PoseStamped  p1;
        p1.pose = currentPose.pose.pose;
        globalPathPub.poses.push_back(p1);

        tf::Quaternion q;
        if (mPointMap.at(mCurrentPoint.first).size() == 2 && mCurrentPoint.second == 0)
        {
            q = tf::createQuaternionFromYaw(-globalGoal.point.yaw);
        }
        else
        {
            q = tf::createQuaternionFromYaw(globalGoal.point.yaw);
        }

        p1.pose.position.x = globalGoal.point.x;
        p1.pose.position.y = globalGoal.point.y;
        //ROS_INFO_STREAM("yaw  "  << globalGoal.point.yaw );
        p1.pose.orientation.x = q.getX();
        p1.pose.orientation.y = q.getY();
        p1.pose.orientation.z = q.getZ();
        p1.pose.orientation.w = q.getW();
        globalPathPub.poses.push_back(p1);
        
        mGlobalPathPub.publish(globalPathPub);
        ///完成发布
        ROS_INFO("+++++++++");
        
        bool isGetGoal = false;
        
        static int pathTimeOutCnt = 0;

        if(mIsGetLocalPath){
            pathTimeOutCnt = 0;
            mIsGetLocalPath = false;
        }else{
            pathTimeOutCnt++;
        }

        if(pathTimeOutCnt > 50){
            ROS_INFO("cnt > 50");
        }

        if (!mLocalPath.poses.empty())
        {
            int goalNumber = getForwardGoal(mLocalPath, odom::getAmclPose());
            goalPose = mLocalPath.poses[goalNumber].pose;
            float dis = getDeltLength(odom::getAmclPose().pose.pose, goalPose);
            if (dis < 0.2)
            {
                goalNumber++;

                if (goalNumber >= mLocalPath.poses.size())
                {
                    isAvoidObstacle = false;
                    mRunState = FPState::RunSellPath;
                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                    //p.point.yaw = 0;// -p.point.yaw;
                    navPointToRosPose(p, goalPose);
                    break;
                }
                goalPose = mLocalPath.poses[goalNumber].pose;
            }
            ROS_INFO_STREAM(goalPose);
            result = runProcessing(out_put_twist, isGetGoal, goalPose);
        }
        else
        {
            result = true;
        }

        cv::Point2d currentPoint(odom::getAmclPose().pose.pose.position.x, odom::getAmclPose().pose.pose.position.y);
        cv::Point2d globalGoalPoint (globalGoal.point.x, globalGoal.point.y);
        if(getDeltLength(currentPoint, globalGoalPoint) < 0.3 ){
                    isAvoidObstacle = false;
                    mRunState = FPState::RunSellPath;
                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                    //p.point.yaw = 0;// -p.point.yaw;
                    navPointToRosPose(p, goalPose);
                    break;
        }
        

#if 0
        if (mIsGetLocalPath)
        {
            int goalNumber = getForwardGoal(mLocalPath, odom::getAmclPose());
            goalPose = mLocalPath.poses[goalNumber].pose;
            float dis = getDeltLength(odom::getAmclPose().pose.pose, goalPose);
            if (dis < 0.2)
            {
                goalNumber++;

                if (goalNumber >= mLocalPath.poses.size())
                {
                    isAvoidObstacle = false;
                    mRunState = FPState::RunSellPath;
                    YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                    navPointToRosPose(p, goalPose);
                    break;
                }
                goalPose = mLocalPath.poses[goalNumber].pose;
            }

            result = runProcessing(out_put_twist, isGetGoal, goalPose);
            mIsGetLocalPath = false;
        }else{
            ROS_INFO("not get path");
            result = true;
        }
#endif




        break;
    }

    default:
    {
        break;
    }
    }
    mPubRunState.publish(pubState);

    if (!isStartOrStopWork)
    {
        out_put_twist = stop();
    }
    geometry_msgs::PoseStamped pubGoal;
    pubGoal.header.frame_id = "map";
    pubGoal.header.stamp = ros::Time::now();
    pubGoal.pose = goalPose;
    mGoalPub.publish(pubGoal);

    return result;
}

bool followpath::runChargeTest(geometry_msgs::Twist& out_put_twist, bool& isStartOrStopWork, int& error_code) 
{
    bool return_bool = true;
    static geometry_msgs::Pose goalPose;
    autodockCtrlData.data = 3;
    static FPState::FollowPathState oldStatus = mRunState;

    if(oldStatus != mRunState){
        ROS_INFO_STREAM("run charge test");
        ROS_INFO_STREAM("mRunState " << mRunState );
        ROS_INFO_STREAM("goal " << goalPose);
    }
    oldStatus = mRunState;

    switch (mRunState)
    {
        case FPState::InitPath: {
            if(!mPointMap.empty()){
                if(mPointMap.count(Yours_Sell_Path_1) > 0){
                    mRunState = FPState::WaitRunCommand;
                    mCurrentPoint.first = Yours_Sell_Path_1;
                    mCurrentPoint.second = 0;
                }
            }else{
                
            }
            out_put_twist = stop();
            break;
        }
        case FPState::WaitRunCommand:{
            if(isStartOrStopWork){
                YoursNavPoint p = mPointMap.at(mCurrentPoint.first)[mCurrentPoint.second];
                navPointToRosPose(p, goalPose);
                    mRunState = FPState::RunSellPath;

            }
            
            out_put_twist = stop();
            break;
        }
        case FPState::RunSellPath:{
            bool isGetGoal = false;
            return_bool = runProcessing(out_put_twist, isGetGoal, goalPose);
            if(isGetGoal){
                mRunState = FPState::RunPrepareCharge;
            }

            break;
        }
        case FPState::RunPrepareCharge:{
            if (turnAngleByWheelOdom(mPreChargeAngle, out_put_twist))
            {
                //isStartOrStopWork = false;
                if (isHaveAutoCharge)
                {
                    mRunState = FPState::AutoDock;
                }
                else
                {
            	    //isStartOrStopWork = false;
                    mRunState = FPState::WaitRunCommand;
                }
            }

            break;
        }
        case FPState::AutoDock:{
            autodockCtrlData.data = 1;
            if(mDockstatus.data == 2){ //充电成功
                //isStartOrStopWork = false;
                autodockCtrlData.data = 4;
                mRunState = FPState::WaitRunCommand;
            }else if(mDockstatus.data == 3){//充电失败
                autodockCtrlData.data = 4;
                mRunState = FPState::WaitRunCommand;
            }

			if(odom::UltrasonicHaveObstacles()){
                autodockCtrlData.data = 4;
                isStartOrStopWork = false;
                mRunState = FPState::WaitRunCommand;
            }

            break;
        }
        case FPState::Charging:{
            if(isStartOrStopWork){
                mRunState = FPState::WaitRunCommand;
            }
            break;
        }

    default:
        break;
    }
    mAutodockCtrlPub.publish(autodockCtrlData);

    if (odom::UltrasonicHaveObstacles()) {
        out_put_twist = stop();
    }

    return return_bool;
}

bool followpath::IsNeedToGoal(YoursNavPoint goal,
                              const double& dis_threshold) {
    nav_msgs::Odometry current_pose = odom::getAmclPose();
    YoursNavPoint current_nav_pose(current_pose);
    YoursRobotPoint current_robot_point = current_nav_pose.point;
    const double distance = goal.point.distance(current_robot_point);
	if(distance > dis_threshold){
        return true;
    }
	return false;
}

bool followpath::IsTurnOk(geometry_msgs::Twist& out_put_twist) { 


}

bool followpath::IsRunOk(geometry_msgs::Twist& out_put_twist) { 


}


//给上层一个指示，地图是否读取完毕
bool followpath::isMapReadOk() {
    bool result = odom::getPoseReady();
    if (!result) {
        std_msgs::Int32 msg;
        msg.data = 703;
        mPubRunState.publish(msg);
    }
    return result;
}

void followpath::pubMapReadNotReady() {
    std_msgs::Int32 msg;
    msg.data = 703;
    mPubRunState.publish(msg);
}