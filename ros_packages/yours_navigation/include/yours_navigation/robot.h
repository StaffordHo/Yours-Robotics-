/*
 * Copyright 2020 <copyright holder> <email>
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

#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define EPSILON 0.000001

enum YoursPath{
    Yours_Path_Null = 0,
    Yours_Sell_To_Charge_Path,
    Yours_Charge_To_Sell_Path,
    Yours_Sell_To_Replenish_Path,
    Yours_Replenish_To_Sell_Path,
    Yours_Sell_Path_1,
    Yours_Sell_Path_2,
    Yours_Sell_Path_3,
    Yours_Sell_Path_4,
    Yours_Sell_Path_5,
    Yours_Sell_Path_6,
    Yours_Sell_Path_7
};

struct YoursRobotPoint{
    YoursRobotPoint():
        x(0.)
      ,y(0.)
      ,yaw(0.)
    {};

    YoursRobotPoint(double x_, double y_, double yaw_):
        x(x_)
      ,y(y_)
      ,yaw(yaw_)
    {};

    ///x 
    double x;
    ///y 
    double y;
    ///弧度值
    double yaw;

    double distance(const YoursRobotPoint& p){
        return sqrt(pow((p.x - this->x),2) + pow((p.y - this->y),2));
    };

};

///Yours Tech

/**
 * @brief 用于描述导航点
 * 每个导航点有位置，朝向，行驶最高速度，停车距离，停车宽度，所在路径等信息。
 * 
*/
struct YoursNavPoint{
    YoursNavPoint():
        point(YoursRobotPoint())
      ,speed(0.3)
      ,stopWidth(0.9)
      ,stopHeight(1.5)
      ,pathName(Yours_Path_Null)
      ,id(-1)
    {};

    YoursNavPoint(const double& x, const double& y, const double& yaw):
        point(YoursRobotPoint(x,y,yaw))
      ,speed(0.3)
      ,stopWidth(0.9)
      ,stopHeight(1.5)
      ,pathName(Yours_Path_Null)
      ,id(-1)
    {};

    YoursNavPoint(const double& x, const double& y, const double& yaw, const double& speed_, const YoursPath& path_):
        point(YoursRobotPoint(x,y,yaw))
      ,speed(speed_)
      ,stopWidth(0.9)
      ,stopHeight(1.5)
      ,pathName(path_)
      ,id(-1)
    {};


    YoursNavPoint(const double& x, const double& y, const double& yaw, const double& speed_, const double& w, const double& h,  const YoursPath& path_, const int& id_):
        point(YoursRobotPoint(x,y,yaw))
      ,speed(speed_)
      ,stopWidth(w)
      ,stopHeight(h)
      ,pathName(path_)
      ,id(id_)
    {};

    YoursNavPoint(const nav_msgs::Odometry& robotPose){
                double rX = robotPose.pose.pose.position.x;
                double rY = robotPose.pose.pose.position.y;
                double rYaw = tf::getYaw(tf::Quaternion(robotPose.pose.pose.orientation.x, robotPose.pose.pose.orientation.y, robotPose.pose.pose.orientation.z, robotPose.pose.pose.orientation.w));
                this->point = YoursRobotPoint(rX, rY, rYaw);
    };

  //p1 to p2 Yaw
    float getYaw(YoursNavPoint p2)
    {
      YoursNavPoint p1 = *this;

      float a = 0, Yaw = 0;
      a = -(p1.point.y - p2.point.y) / (p1.point.x - p2.point.x);
      if (std::isnan(a))
      {
        if (p1.point.y > p2.point.y)
          Yaw = -1.57;
        else
          Yaw = 1.57;
      }
      else
      {
        if (a < 0)
        {
          Yaw = -atanf(a);
          if (p1.point.y > p2.point.y)
          {
            if (Yaw > 0)
              Yaw = Yaw + M_PI;
            else
              Yaw = Yaw - M_PI;
          }
        }
        else
        {
          Yaw = -atanf(a);
          if (p1.point.y < p2.point.y)
          {
            if (Yaw > 0)
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

    YoursRobotPoint point;
    double speed;
    double stopWidth;
    double stopHeight;
    YoursPath pathName;
    int id;
};



class Robot
{
public:
Robot();
~Robot();
};

struct rPoint{
	rPoint():x(.0),y(.0){};
	rPoint(const double& x_, const double& y_):x(x_),y(y_){};
	double x;
	double y;
	double distanceTo(const rPoint& p){
		double x1 = this->x, y1 = this->y;
		double x2 = p.x, y2 = p.y;
		double dis = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		return dis;
	}
};



struct rZone{
  std::vector<rPoint> points;

  void printZone(){
    int pointCnt = 0;
    for(auto i:points){
      std::cout<< "point " << pointCnt <<  " x " << i.x << " y " << i.y << std::endl;
      pointCnt++;
    }

  }

};

struct rLine{
    rLine(){};
    rLine(rPoint p1_, rPoint p2_):p1(p1_), p2(p2_){

    };

    rPoint p1;
    rPoint p2;
   /*
    double distanceToPoint(const rPoint &inPoint)
    {
      rPoint pt3 = inPoint;
      double A = (p1.y - p2.y) / (p1.x - p2.x);
      double B = (p1.y - A * p1.y);
      /// > 0 = ax +b -y;
      return fabs(A * pt3.x + B - pt3.y) / sqrt(A * A + 1);
    }
    */



/*********************************/
// 如果经过点做直线的垂足，垂足落在线段上，则取垂线段的距离
// 否则取到线段两端点距离的最小值
//
// 参数：
// point:  存储点的xy坐标
// p1, p2: 线段的两点
//
// return: 点到线段的最小距离



//float CalculatePointToLineDistance(rPoint point, const rPoint& p1, const rPoint& p2)
    double distanceToPoint(const rPoint &point)
{
    float dis = 0.f;
 
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
 
    // 两直线垂直，向量表示法，转换后公示
    float k = -((p1.x - point.x)*dx + (p1.y - point.y)*dy) / ( dx*dx + dy*dy);
    float footX = k*dx + p1.x;
    float footY = k*dy + p1.y;
 
    //if垂足是否落在线段上
    if( footY >= std::min(p1.y, p2.y) && footY <=std::max(p1.y, p2.y)
        && footX >= std::min(p1.x, p2.x) && footX <=std::max(p1.x, p2.x ) )
    {
        dis = sqrtf((footX-point.x)*(footX-point.x) + (footY-point.y)*(footY-point.y));
   }
    else 
    {
        float dis1 = sqrtf((p1.x-point.x)*(p1.x-point.x) + (p1.y-point.y)*(p1.y-point.y));
        float dis2 = sqrtf((p2.x-point.x)*(p2.x-point.x) + (p2.y-point.y)*(p2.y-point.y));
 
        dis = ( dis1 < dis2 ? dis1 : dis2 );
    }
 
    return dis;

}


  

};


#endif // ROBOT_H
