#ifndef EXECUTE_ROBOT_RETURN_H
#define EXECUTE_ROBOT_RETURN_H

#include <ros/ros.h>
#include <vector>
#include <memory>
#include "robot.h"

/**
 * @brief 本class用于计算机器人执行返回命令时的返回路径
 * 根据当前机器人所处在的任务状态和位置，计算返回路径。
 * 
 */


class ExecuteRobotReturn{




public:
ExecuteRobotReturn();
~ExecuteRobotReturn();


    /**
     * @brief Get the Robot Sell Path Return Path object
     * 
     * @param pathMap 全局路径点
     * @param currentPoint 机器人所在的当前路径和当前目标点 
     * @param outReturnPath 输出的返回路径，机器人要按照这个路径行走
     * @return true 
     * @return false 
     */
    static bool getRobotSellPathReturnPath(const std::map<YoursPath, std::vector<YoursNavPoint>> &pathMap,
                            const std::pair<YoursPath, int>& currentPoint,
                            const YoursNavPoint& robotPose,
                            const bool& isToChargePath,
                            std::vector<YoursNavPoint>& outReturnPath,
                            bool& isForward
                            );

    static bool getRobotToSellPathReturnPath(const std::map<YoursPath, std::vector<YoursNavPoint>> &pathMap,
                            const YoursNavPoint& robotPose,
                            std::pair<YoursPath, int> &currentPoint);




};


#endif
