//
// Created by jiangbo on 2019/12/16.
//

#ifndef YOURS_NAVIGATION_LOSEPROCESSING_H
#define YOURS_NAVIGATION_LOSEPROCESSING_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#define TURNCNT 50

enum LoseProcessingTurnStatus{
    TurnLeft,
    LeftToCenter,
    TurnRight,
    RightToCenter
};

enum RelocationStatus{
    RelocationTry,
    RelocationTurn
};

class LoseProcessing {
    bool isTurnLeft;
    geometry_msgs::Twist mTwist;
    int turnCnt;
    LoseProcessingTurnStatus mTurnStatus;
    geometry_msgs::Twist stop();
    RelocationStatus mRelocationStatus;
public:
    LoseProcessing();
    ~LoseProcessing();
    int processingLose(bool inHaveObstacles, bool inHaveUserScan, bool isLose,geometry_msgs::Twist& outTwist);
    bool processingRelocal(const double& goalYaw, const double& currentYaw, geometry_msgs::Twist& outTwist);
    int setProcessingEnd(bool isEnd);

};


#endif //YOURS_NAVIGATION_LOSEPROCESSING_H
