#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <yours_message/TestNextGoal.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
//测试程序设计
//1.数据来源发送的路径数据，初始点在充电处
//2.运行方式，接收到速度指令，等几个周期就向前走一格，自己定义运行步进
//3.运行轨迹发布到rviz 用于观察运行

geometry_msgs::PoseArray mPoseArray;
std::map<int, geometry_msgs::PoseArray> mPoseArrayMap;
bool isGetGoal = false;

void yoursGoalCallBack(const geometry_msgs::PoseArrayConstPtr& msg);
void yoursNextGoalCallBack(const yours_message::TestNextGoal& msg);

std::pair<int,int> mNextGoal;
std::pair<int,int> mNewNextGoal;
bool isGetNextGoal = false;
bool isStartWork = false;

int main(int argc, char** argv){
    ros::init(argc, argv, "yours_navigation_test");
    ros::NodeHandle n("~");
    
    ros::Subscriber subGoal = n.subscribe("/yours_goal_manual_set", 5, &yoursGoalCallBack);
    ros::Subscriber nextGoal = n.subscribe("/yours_test_next_goal", 5, &yoursNextGoalCallBack);
    ros::Publisher pubGoal = n.advertise<geometry_msgs::PoseArray>("/yours_check_goal", 1);
    
   
    //orb interface
    ros::Publisher slamStatePub = n.advertise<std_msgs::Int32>("/yours_slam_state",1);
    ros::Publisher robotPosePub = n.advertise<geometry_msgs::PoseStamped>("/yours_camera",1);
    ros::Publisher startWorkPub = n.advertise<std_msgs::Bool>("/start_work", 1);
    
//    cv::Mat ctrlImage = cv::imread("/home/jiangbo/imageDelay/mod.jpeg");
    cv::Mat ctrlImage = cv::Mat(400, 400, CV_8UC3, cv::Scalar::all(0));
    
    ros::Rate rate(5);
    while(ros::ok()){
        
        if(isGetNextGoal){
            isGetNextGoal = false;
        }
        if(!isStartWork){
            if(isGetGoal){
                geometry_msgs::PoseStamped orbPose;
                geometry_msgs::PoseArray poseArray = mPoseArrayMap.at(2);
                geometry_msgs::Pose pose = poseArray.poses[2];
                orbPose.header.frame_id = "/odom";
                orbPose.header.stamp = ros::Time::now();
                orbPose.pose = pose;
                robotPosePub.publish(orbPose);
                mNextGoal.first = 2;
                mNextGoal.second = 2;
            }
        }
        
        if(isGetGoal){
            if(mPoseArrayMap.count(2)){
                if(mPoseArrayMap.at(2).poses.size() > 0){
                    std_msgs::Int32 slamState;
                    slamState.data = 2;
                    slamStatePub.publish(slamState);
                }
                if(isStartWork){
                    geometry_msgs::PoseStamped orbPose;
                    geometry_msgs::PoseArray poseArray = mPoseArrayMap.at(mNextGoal.first);
                    geometry_msgs::Pose pose = poseArray.poses[mNextGoal.second];
                    orbPose.header.frame_id = "/odom";
                    orbPose.header.stamp = ros::Time::now();
                    orbPose.pose = pose;
                    robotPosePub.publish(orbPose);
                }
            }
        }
        
        if(isGetGoal)
            cv::imshow("image", ctrlImage);
        
        int key = cv::waitKey(30);
        if(key == 'a'){
            ROS_INFO("pess a");
            if(isStartWork){
                //nav_msgs::Odometry orbPose;
               /*
                geometry_msgs::PoseStamped orbPose;
                geometry_msgs::PoseArray poseArray = mPoseArrayMap.at(mNextGoal.first);
                geometry_msgs::Pose pose = poseArray.poses[mNextGoal.second];
                orbPose.header.frame_id = "/odom";
                orbPose.header.stamp = ros::Time::now();
                orbPose.pose = pose;
                robotPosePub.publish(orbPose);
                */
               mNextGoal = mNewNextGoal;
            }
        }else if(key == 's'){
            ROS_INFO("pess s");
            std_msgs::Int32 slamState;
            std_msgs::Bool startWork;
            slamState.data = 2;
            startWork.data = true;
            isStartWork = true;
            startWorkPub.publish(startWork); 
            
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;    
}


void yoursNextGoalCallBack(const yours_message::TestNextGoal& msg){
    
    int f = (int)msg.path_code.data;
    
    mNewNextGoal.first = (int)msg.path_code.data;
    mNewNextGoal.second = (int)msg.pose_code.data;
    isGetNextGoal = true;
    ROS_INFO("get Path Code %d ,get Pose Code %d", mNewNextGoal.first, mNewNextGoal.second);
    
}


void yoursGoalCallBack(const geometry_msgs::PoseArrayConstPtr& msg){
    
    mPoseArray.poses.clear();
    for(int i = 0; i < msg->poses.size(); i++){
        mPoseArray.poses.push_back(msg->poses[i]);
    }
  
    std::string c_string = msg->header.frame_id;
    //std::string = std::string( id.data.c_str() );
    if(c_string.compare("0") == 0){
        mPoseArrayMap.clear();
    }
    int id_int = std::atoi(c_string.c_str());
    mPoseArrayMap[id_int] = mPoseArray;
    if(c_string.compare("4") ==  0){
        for(auto it:mPoseArrayMap){
            std::cout << it.first << std::endl;
            geometry_msgs::PoseArray data = it.second;
            std::cout << data.poses.size() << std::endl;
            for(int i = 0; i < data.poses.size() ;i++){
                ROS_INFO_STREAM ("data " << data.poses[i] );
            } 
            
            //for(auto itsub:it. ){
            //    ROS_INFO_STREAM ("data " << itsub );
                
            //}
        }
        ROS_INFO(" test get goal ");
        isGetGoal = true;
    }
    
    
}