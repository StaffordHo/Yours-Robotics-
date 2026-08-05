#include "../include/yours_navigation/execute_robot_return.h"


int findNearestPoint(const std::vector<YoursNavPoint>&path_, const YoursNavPoint&point_ ){
    float minDis = 999.0;
    int nearestID = 0;
    for(int i = 0; i < path_.size(); i++){
        YoursNavPoint p = path_[i];
        double dis = p.point.distance(point_.point);
        if(dis < minDis){
            minDis = dis;
            nearestID = i;
        }
    }
    return nearestID;
}

double getToLinkPointDis(const std::vector<YoursNavPoint> &path_,
                         const YoursNavPoint &inCurrentPoint_,
                         const int &goalID_,
                         const int &linkID_,
                         const bool &forward,
                         std::vector<int> &returnPathVector_)
{
    int currentID = 0;
    int goalID = goalID_;
    YoursNavPoint currentPoint = inCurrentPoint_;
    YoursNavPoint goalPoint = path_[goalID];
    double disAll = 0.0;
    returnPathVector_.clear();
    for (;;)
    {
        returnPathVector_.push_back(goalID);
        double dis = currentPoint.point.distance(goalPoint.point);
        disAll = disAll + dis;
        currentID = goalID;
        currentPoint = path_[currentID];
        if (goalID == linkID_)
        {
            //若到达了链接点，则完成计算
            break;
        }
        if (forward)
        {
            goalID ++;
        }
        else
        {
            goalID --;
        }

        if (goalID >= int(path_.size()))
        {
            goalID = 0;
        }
        
        if (goalID <= -1)
        {
           goalID = path_.size() - 1;
        }
        goalPoint = path_[goalID];
    }
    return disAll;
}

ExecuteRobotReturn::ExecuteRobotReturn(){

}

ExecuteRobotReturn::~ExecuteRobotReturn(){
}

bool ExecuteRobotReturn::getRobotSellPathReturnPath(const std::map<YoursPath, std::vector<YoursNavPoint>> &pathMap,
                                                    const std::pair<YoursPath, int> &currentPoint,
                                                    const YoursNavPoint &robotPose,
                                                    const bool &isToChargePath,
                                                    std::vector<YoursNavPoint> &outReturnPath,
                                                    bool& isForward
                                                    )
{
    if(currentPoint.first != Yours_Sell_Path_1){
        return false;
    }
    YoursPath goToPath = Yours_Sell_To_Charge_Path ;
    if(!isToChargePath){
        if (pathMap.count(Yours_Sell_To_Replenish_Path) > 0)
        {
            goToPath = Yours_Sell_To_Replenish_Path;
        }
    }

    if(pathMap.count( goToPath ) <= 0){
        return false;
    }

    YoursNavPoint goalPoint = pathMap.at(currentPoint.first)[currentPoint.second];
    YoursNavPoint passGoalPoint;
    int passGoalID = 0;
    if(currentPoint.second == 0){
        passGoalPoint = pathMap.at(currentPoint.first)[pathMap.at(currentPoint.first).size() - 1];
        passGoalID = pathMap.at(currentPoint.first).size() - 1;
    }else{
        passGoalPoint = pathMap.at(currentPoint.first)[currentPoint.second - 1];
        passGoalID = currentPoint.second - 1;
    }
    YoursNavPoint sellToChargeStartPoint = pathMap.at( goToPath )[0];
    int linkID = findNearestPoint(pathMap.at(Yours_Sell_Path_1), sellToChargeStartPoint);
    //linkID = 0; 
    //开始计算正向和反向行走到链接点的距离
    std::vector<int> forwardGoalIDVector;
    std::vector<int> backwardGoalIDVector;
    double forwardDistance = getToLinkPointDis(pathMap.at(Yours_Sell_Path_1), robotPose, currentPoint.second, linkID, true, forwardGoalIDVector);
    double backwardDistance = getToLinkPointDis(pathMap.at(Yours_Sell_Path_1), robotPose, passGoalID, linkID, false, backwardGoalIDVector);
    //判断哪个路径更短，我们需要走短的那个路径
    outReturnPath.clear();
    std::vector<int> finialVector;
    isForward = true;
    if(forwardDistance < backwardDistance){
        finialVector = forwardGoalIDVector;
    }else{
        finialVector = backwardGoalIDVector;
        isForward = false;
    }

    for (auto it : finialVector)
    {
        outReturnPath.push_back(pathMap.at(Yours_Sell_Path_1)[it]);
    }

    return true;
}

bool ExecuteRobotReturn::getRobotToSellPathReturnPath(const std::map<YoursPath, std::vector<YoursNavPoint>> &pathMap,
                                                      const YoursNavPoint &robotPose,
                                                      std::pair<YoursPath, int> &currentPoint)
{
    if((currentPoint.first != Yours_Charge_To_Sell_Path) && (currentPoint.first != Yours_Replenish_To_Sell_Path)){
        ROS_INFO_STREAM("(currentPoint.first != Yours_Charge_To_Sell_Path) && (currentPoint.first != Yours_Replenish_To_Sell_Path)");
        return false;
    }

    if(pathMap.count(Yours_Charge_To_Sell_Path) <= 0 && pathMap.count(Yours_Replenish_To_Sell_Path) <= 0 ){
        ROS_INFO_STREAM("(Yours_Charge_To_Sell_Path) <= 0 || pathMap.count(Yours_Replenish_To_Sell_Path)<= 0");
        return false;
    }

    //returnMap.clear();

    int return1stPointID = 100;
    if(pathMap.count(Yours_Replenish_To_Sell_Path) > 0){
    }
    else
    {
        ROS_INFO_STREAM(" x y yaw  " << robotPose.point.x << "   " <<robotPose.point.y << "   " << robotPose.point.yaw);

        ///没有补货路径，仅仅检查充电路径就好
        int nearestPointID = findNearestPoint(pathMap.at(Yours_Sell_To_Charge_Path), robotPose);
        int passNearstPointID = nearestPointID + 1;
        ROS_INFO_STREAM(" ID  pID " << nearestPointID << "   " << passNearstPointID);
        if (nearestPointID == pathMap.at(Yours_Sell_To_Charge_Path).size() - 1)
        {
            //returnMap.push_back(pathMap.at(Yours_Sell_To_Charge_Path)[nearestPointID]);
            currentPoint.first = Yours_Sell_To_Charge_Path;
            currentPoint.second = nearestPointID; 
            return true;
        }

        YoursNavPoint goalPoint = pathMap.at(Yours_Sell_To_Charge_Path)[nearestPointID];
        YoursNavPoint passGoalPoint = pathMap.at(Yours_Sell_To_Charge_Path)[passNearstPointID];
        double distanceTwoGoalPoint = goalPoint.point.distance(passGoalPoint.point);
        double distanceCurrentToPassGoalPoint = passGoalPoint.point.distance(robotPose.point);
        return1stPointID = nearestPointID;
        if (distanceCurrentToPassGoalPoint < distanceTwoGoalPoint)
        {
            return1stPointID = passNearstPointID;
        }
        currentPoint.first = Yours_Sell_To_Charge_Path;
        currentPoint.second = return1stPointID ; 
/*
        for (int i = return1stPointID; i < pathMap.at(Yours_Sell_To_Charge_Path).size(); i++)
        {
            returnMap.push_back(pathMap.at(Yours_Sell_To_Charge_Path)[i]);
        }
        */
    }

    return true;
}
