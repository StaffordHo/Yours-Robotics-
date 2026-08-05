//
// Created by jiangbo on 2019/12/16.
//

#include "../include/yours_navigation/LoseProcessing.h"

LoseProcessing::LoseProcessing():turnCnt(TURNCNT),isTurnLeft(true),mTurnStatus(TurnLeft) {

}

LoseProcessing::~LoseProcessing(){


}

geometry_msgs::Twist LoseProcessing::stop(){
    geometry_msgs::Twist out;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = 0.0;
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    return out;
}

geometry_msgs::Twist left(float speed=0.1){
    geometry_msgs::Twist out;
    out.angular.x = 0.0;
    out.angular.x = 0.0;
    out.angular.z = speed;
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    return out;
}

geometry_msgs::Twist right(){
    geometry_msgs::Twist out;
    out.angular.x = 0.0;
    out.angular.x = 0.0;
    out.angular.z = -0.1;
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    return out;
}


int LoseProcessing::processingLose(bool inHaveObstacles, bool inHaveUserScan, bool isLose, geometry_msgs::Twist& outTwist){
    if(isLose) {
        if (inHaveObstacles) {
            mTwist = this->stop();
        } else {
            if (inHaveUserScan) {
                mTwist = this->stop();
            } else {
                switch (mTurnStatus) {
                    case TurnLeft: {
                        mTwist = left();
                        turnCnt--;
                        if (turnCnt < 0) {
                            turnCnt = TURNCNT;
                            mTurnStatus = LeftToCenter;
                        }
                        break;
                    }
                    case LeftToCenter: {
                        mTwist = right();
                        turnCnt--;
                        if (turnCnt < 0) {
                            turnCnt = TURNCNT;
                            mTurnStatus = TurnRight;
                        }
                        break;
                    }
                    case TurnRight: {
                        mTwist = right();
                        turnCnt--;
                        if (turnCnt < 0) {
                            turnCnt = TURNCNT;
                            mTurnStatus = RightToCenter;
                        }
                        break;
                    }
                    case RightToCenter: {
                        mTwist = left();
                        turnCnt--;
                        if (turnCnt < 0) {
                            turnCnt = TURNCNT;
                            mTurnStatus = TurnLeft;
                        }
                        break;
                    }
                }
            }
        }
        outTwist = mTwist;
    }
    mRelocationStatus = RelocationTry;
}

int LoseProcessing::setProcessingEnd(bool isEnd){
    turnCnt = TURNCNT;
    mTurnStatus = TurnLeft;
}

bool LoseProcessing::processingRelocal(const double& goalYaw, const double& currentYaw, geometry_msgs::Twist& outTwist){
    static int tryCnt = 0;
    static double tryStartYaw = 0.0;
    static float turnSpeed = 0.1;
    if(fabs(currentYaw - goalYaw) < 0.1 ){
        tryCnt = 10;
        mRelocationStatus = RelocationTry;
        return true;
    }else{
        switch (mRelocationStatus){
            case RelocationTry: {
                if(tryCnt == 10){
                    tryStartYaw = fabs(currentYaw - goalYaw);
                }
                if(tryCnt > 0) {
                    outTwist = left(0.1);
                    tryCnt--;
                }else{
                    if(fabs(currentYaw - goalYaw) < tryStartYaw){
                        turnSpeed = 0.1;
                    }else{
                        turnSpeed = -0.1;
                    }
                    mRelocationStatus = RelocationTurn;
                }
                break;
            }
            case RelocationTurn:{
                tryCnt = 10;
                outTwist = left(turnSpeed);
                if(fabs(currentYaw - goalYaw) < tryStartYaw){
                    ROS_INFO("EEEERRRRR RelocationTurn");
                    turnSpeed = 0 - turnSpeed;
                }

                break;
            }
        }

        if(currentYaw > goalYaw)
          outTwist = left();
        else
          outTwist = right();
        return false;
    }
}
