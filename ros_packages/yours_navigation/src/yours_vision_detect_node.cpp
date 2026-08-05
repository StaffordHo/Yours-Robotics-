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

#include "../include/yours_navigation/yours_vision_detect_node.h"
#include <cv_bridge/cv_bridge.h>


/*
  <param name="vision_width_threshold" value="0.6"/> 
  <param name="vision_farthest_threshold" value="0.8"/>
  <param name="vision_height_threshold" value="0.75"/>
  <param name="vision_low_threshold" value="0.3"/>
*/

cv::Point clickPoint(0,0);

void mouseCallback(int event, int x, int y, int flags, void* param){
    if(event == cv::EVENT_LBUTTONUP){
        clickPoint = cv::Point(x, y);
        ROS_INFO("x %d  y %d", x, y);
    }
    
}


yours_vision_detect_node::yours_vision_detect_node(ros::NodeHandle& n_):
mObstaclesPixel(0)
,mIsDebugOut(false)
{
    mN = n_;
//    colorSub = n_.subscribe("/d435/color/image_raw", 1, &yours_vision_detect_node::colorImageCallback, this);
    depthSub = n_.subscribe("/d435/depth/image_rect_raw", 1, &yours_vision_detect_node::depthImageCallback, this);
//    infraSub = n_.subscribe("/d435/infra1/image_rect_raw", 1, &yours_vision_detect_node::infraImageCallback, this);
    depthInfoSub = n_.subscribe("/d435/depth/camera_info", 1, &yours_vision_detect_node::depthImageInfoCallback, this);
/*    
    colorSub = n_.subscribe("/d435i/color/image_raw", 1, &yours_vision_detect_node::colorImageCallback, this);
    depthSub = n_.subscribe("/d435i/depth/image_rect_raw", 1, &yours_vision_detect_node::depthImageCallback, this);
    infraSub = n_.subscribe("/d435i/infra1/image_rect_raw", 1, &yours_vision_detect_node::infraImageCallback, this);
    depthInfoSub = n_.subscribe("/d435i/depth/camera_info", 1, &yours_vision_detect_node::depthImageInfoCallback, this);
    */
	debugImagePub = n_.advertise<sensor_msgs::Image>("/yours_vision_detect_debug_image", 1);

    n_.param("vision_width_threshold", mVisionWidthThreshold, double(0.6));
    n_.param("vision_farthest_threshold",mVisionFarthestThreshold, double(0.8));
    n_.param("vision_height_threshold", mVisionHeightThreshold, double(0.75));
    n_.param("vision_low_threshold", mVisionLowThreshold, double(0.35));
    n_.param("vision_detect_pixel", mVisionDetectThreshold, int(10000));
    n_.param("vision_debug_out", mIsDebugOut, false);
    n_.param("camera_angle", mCameraAngle, double(30.0));

    depthInfo.clear();
    //cv::namedWindow("depth");
    //cv::setMouseCallback("depth", &mouseCallback, 0);
}

yours_vision_detect_node::~yours_vision_detect_node()
{

}

void yours_vision_detect_node::depthImageCallback(const sensor_msgs::Image& msgs)
{
    /*
    ROS_INFO("depth");
    ROS_INFO("%d %d %d", msgs.width, msgs.height, msgs.step);
    ROS_INFO("data size = %d ", msgs.data.size());
    */
    depthMat = cv::Mat(msgs.height, msgs.width, CV_16UC1, cv::Scalar::all(0));
    //depthMat.data = msgs.data.data();
    msgs.data[1];
    
    depthMat = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::TYPE_16UC1)->image;

}
void yours_vision_detect_node::colorImageCallback(const sensor_msgs::Image& msgs)
{
    /*
    ROS_INFO("color");
    ROS_INFO("%d %d %d", msgs.width, msgs.height, msgs.step);
    ROS_INFO("encode = %d ", msgs.encoding);
    */
    colorMat = cv_bridge::toCvCopy(msgs, "bgr8")->image;
	//cv::cvtColor(colorMat , infraMat , CV_BGR2GRAY);
	//infraMat = cv_bridge::toCvCopy(msgs, "gray")->image;
}

void yours_vision_detect_node::infraImageCallback(const sensor_msgs::Image& msgs)
{
    infraMat = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::TYPE_8UC1)->image;
}

void yours_vision_detect_node::depthImageInfoCallback(const sensor_msgs::CameraInfo& msgs)
{
    depthInfo.clear();
    depthInfo.push_back(msgs.K[0]);
    depthInfo.push_back(msgs.K[4]);
    depthInfo.push_back(msgs.K[2]);
    depthInfo.push_back(msgs.K[5]);
    //ROS_INFO_STREAM(msgs.K[0]); //cx
    //ROS_INFO_STREAM(msgs.K[4]); //cy
    //ROS_INFO_STREAM(msgs.K[2]); //fx
    //ROS_INFO_STREAM(msgs.K[5]); //fy
    //ROS_INFO("-----------------");

}


void yours_vision_detect_node::imageProcess(cv::Mat& imageIn){
   cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
   cv::Mat thresh;
   cv::threshold(imageIn, thresh, 5, 1, CV_THRESH_BINARY);
   cv::Mat open_result;
   cv::morphologyEx(thresh, open_result, cv::MORPH_OPEN, kernel);
   //cv::imshow("thresh", thresh);
   //cv::imshow("result", open_result);
   
//cv::cvtColor(imageIn, imageIn, CV_BGR2GRAY);
   
   imageIn = imageIn.mul(open_result)  ;
   //cv::imshow("resultImageR", );
}

int noZeroPixelNumber(cv::Mat& imageIn){
    int number = 0;
    for(int y = 0; y < imageIn.rows; y++){
        uint8_t* imageRow = imageIn.ptr<uint8_t>(y);
        for(int x = 0; x < imageIn.cols; x++){
            if(imageRow[x] != 0){
                number++;
            }
        }
    }
    return number;
}

int yours_vision_detect_node::getDetectPixelQuantity(float imageRowRate, int farthest)
{
    if((!infraMat.empty()) && (!depthMat.empty())){
        for(int y = 0; y < depthMat.rows; y++){
            uint16_t* depthRows = depthMat.ptr<uint16_t>(y);
            uint8_t* infraRows = infraMat.ptr<uint8_t>(y);
            for(int x =0; x < depthMat.cols; x++){
                if(depthRows[x] > farthest || depthRows[x] == 0){
                    infraRows[x] = 0;
                }
                
                if(y >  int(float(depthMat.rows) * (imageRowRate))){
                    infraRows[x] = 0;
                }
            }
        }
    }
    
    int pixel = 0;
    
    if(!infraMat.empty()){
        imageProcess(infraMat);
        pixel = noZeroPixelNumber(infraMat);
        //cv::imshow("infra_cutout", infraMat);
        //cv::waitKey(10);
    }
    
    if(infraMat.empty() || depthMat.empty())
        return -1;
    
    return pixel;
}

bool yours_vision_detect_node::haveObstacle()
{
    //ROS_INFO_STREAM("mObstaclesPixel " << mObstaclesPixel );
    if (mObstaclesPixel > mVisionDetectThreshold)
    {
        return true;
    }
    return false;
}

bool yours_vision_detect_node::run(int& pixel_)
{
    if(!colorMat.empty()){
        //cv::imshow("image", colorMat);
    }
    
    if(!depthMat.empty()){
        //cv::imshow("depth", depthMat);
        //imageProcess(depthMat);
    //cv::resize(depthMat, depthMat, cv::Size(depthMat.cols / 2.0, depthMat.rows / 2.0));
    infraMat = cv::Mat(depthMat.rows, depthMat.cols, CV_8UC1, cv::Scalar::all(0));	
    }
    
   
#if 1
    double sin_y = std::sin(mCameraAngle*M_PI/180.0);
    double cos_z = std::cos(mCameraAngle*M_PI/180.0);
    if ((!infraMat.empty()) && (!depthMat.empty()))
    {
	    mObstaclesPixel = 0;
	    for(int y = 0; y < depthMat.rows; y++){
		    uint16_t* depthRows = depthMat.ptr<uint16_t>(y);
		    uint8_t* infraRows = infraMat.ptr<uint8_t>(y);
		    for(int x =0; x < depthMat.cols; x++){
			    if(!depthInfo.empty() &&  (x%2 == 0) && (y%2 == 0) ){
				    double zz = (depthRows[x]/1000.0) ;
				    double xx = (x - depthInfo[2]) * zz / depthInfo[0];
				    double yy = (y - depthInfo[3]) * zz / depthInfo[1];
				    //yy = yy + (zz * (mCameraAngle*M_PI/180.0));
				    //zz = zz * cos(mCameraAngle*M_PI/180.0);
				    yy = yy + (zz * sin_y);
				    zz = zz * cos_z;
				    if( ( xx > mVisionWidthThreshold/2.0) ||( zz < 0.1 ) || ( xx < -mVisionWidthThreshold/2.0) || ( zz > mVisionFarthestThreshold) ||  ( yy > mVisionLowThreshold) ||( yy < -mVisionHeightThreshold)  ){
				    }else{
					    mObstaclesPixel++;
					    infraRows[x] = 255;
				    } 

			    }   

			    /*

			       if(depthRows[x] > 800 || depthRows[x] == 0){
			       infraRows[x] = 0;
			       }

			       if(y >  int(float(depthMat.rows) * (9.0/10.0))){
			       infraRows[x] = 0;
			       }
			       */
		    }
	    }
	    mObstaclesPixel = mObstaclesPixel * 4; 
    }
#endif
#if 1
    if (mIsDebugOut)
    {
        if (!infraMat.empty())
        {
            
            if (mObstaclesPixel > mVisionDetectThreshold)
            {
                cv::putText(infraMat, std::to_string(mObstaclesPixel), cv::Point(50, 100), 1, 3, cv::Scalar(200));
            }
            else
            {
                cv::putText(infraMat, std::to_string(mObstaclesPixel), cv::Point(50, 100), 1, 3, cv::Scalar(200));
            }
            /*
            cv::imshow("infra_cutout", infraMat);
            */
            sensor_msgs::Image rosImage;
            cv_bridge::CvImage cvi;
            cvi.image = infraMat.clone();
            cvi.header.frame_id = "image";
            cvi.encoding = "mono8";
            cvi.header.stamp = ros::Time::now();
            cvi.toImageMsg(rosImage);
            debugImagePub.publish(rosImage); 
        }
        //ROS_INFO_STREAM("mObstaclesPixel " << mObstaclesPixel);
        //cv::waitKey(10);
    }
#endif
    pixel_ = mObstaclesPixel;
    if(mVisionDetectThreshold < 0){
        return false;
    }

    if (mObstaclesPixel > mVisionDetectThreshold)
    {
        return true;
    }
    return false;


}

void yours_vision_detect_node::setFarthest(double data){
    if (data < 0.8){
        mVisionFarthestThreshold = data;
    }else{
        mVisionFarthestThreshold = 0.8;
    }
}
