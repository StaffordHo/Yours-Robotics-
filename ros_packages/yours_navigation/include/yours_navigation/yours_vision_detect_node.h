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

#ifndef YOURS_VISION_DETECT_NODE_H
#define YOURS_VISION_DETECT_NODE_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


class yours_vision_detect_node
{
   void depthImageCallback(const sensor_msgs::Image& msgs);
   void colorImageCallback(const sensor_msgs::Image& msgs);
   void infraImageCallback(const sensor_msgs::Image& msgs);
   void depthImageInfoCallback(const sensor_msgs::CameraInfo& msgs);
   void imageProcess(cv::Mat& imageIn);
   ros::Subscriber depthSub;
   ros::Subscriber colorSub;
   ros::Subscriber infraSub;
   ros::Subscriber depthInfoSub;
   ros::Publisher debugImagePub;
   cv::Mat depthMat;
   cv::Mat colorMat;
   cv::Mat infraMat;
   ros::NodeHandle mN;
   std::vector<double> depthInfo;


   double mVisionWidthThreshold;
   double mVisionFarthestThreshold;
   double mVisionHeightThreshold;
   double mVisionLowThreshold;
   int mVisionDetectThreshold;
   int mObstaclesPixel;
   double mCameraAngle;

   bool mIsDebugOut;
   int getDetectPixelQuantity(float imageRowRate, int farthest);

public:
    yours_vision_detect_node(ros::NodeHandle& n_);
    ~yours_vision_detect_node();
    /**
     * @brief 运行循环，需要放在一个循环线程中
     * 
     * @param pixel_ 返回数据，输出当前检测到障碍物包含多少像素
     * @return true 有障碍物返回
     * @return false 无障碍物返回
     */
    bool run(int& pixel_);
    /**
     * @brief 有无障碍查询，返回结果逻辑和运行循环(run(*))一致
     * 
     * @return true 有障碍物 
     * @return false 无障碍物
     */
    bool haveObstacle();
    /**
     * @brief Set the Farthest object
     * 随时修改检测的障碍物最远距离，当这个值大于0.8时，最远障碍物距离等于0.8
     * @param data 
     */
    void setFarthest(double data);
   // int tetsFoo(float imageRowRate);
};

#endif // YOURS_VISION_DETECT_NODE_H
