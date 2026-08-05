#ifndef YOURS_LIDAR_DETECT_HUMAN_H_
#define YOURS_LIDAR_DETECT_HUMAN_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
//#include <yours_robot_tools/yours_log.h>
#include <memory>
namespace yours_lidar_detect_human
{
    struct ypoint
    {
        ypoint():x(0.0), y(0.0), angle_(0.0), range_(0.0){

        };

        ~ypoint(){

        };
        double x;
        double y;
        double angle_;
        double range_;
        bool setXYPoint(double angle, double range)
        {
            //double angle = mLaser.angle_min + (i * mLaser.angle_increment);
            //double x = cos(angle) * mLaser.ranges[i];
            //double y = sin(angle) * mLaser.ranges[i];
            //double disn = mLaser.ranges[i];
            ///过滤太近的点
            if (range > 0.01 && range < 0.05)
            {
                return false;
            }

            if(std::isinf(range)){
                return false;
            }

            if(std::isnan(range)){
                return false;
            }

            this->x = cos(angle) * range;
            this->y = sin(angle) * range;
            this->angle_ = angle;
            this->range_ = range;
            //std::cout << " setXY " <<this->x << " " << this->y << " a :" << angle << " r: " << range << std::endl ;
            return true;
        }

        double getDistance(const ypoint& p2){
            //std::cout << this->x << " " << this->y << " " << p2.x << " " << p2.y << std::endl;
            return sqrt(((x - p2.x)*(x - p2.x)) + ((y - p2.y)*(y - p2.y)));
        }

        /* data */
    };

    struct ycluster{
        ycluster():isSlow(false), isStop(false), isNearlyWall(false), isHuman(false), isEyeCandidacy(false), eyeNumber(0), x2lidar(0.0), isNearestRobot4Eye(false){};
        std::vector<ypoint> cluster_;
        bool isSlow;
        bool isStop;
        bool isNearlyWall;
        bool isHuman;
        bool isEyeCandidacy; //eye 候选
        int eyeNumber;
        float x2lidar; //x轴方向到雷达距离
        bool isNearestRobot4Eye;
        void clear() {
            this->cluster_.clear();
            isSlow = false;
            isStop = false;
            isNearlyWall = false;
            isHuman = false;
            isEyeCandidacy = false;
            eyeNumber = 0;
            x2lidar = 0.0;
            isNearestRobot4Eye = false;
        }
    };

    struct ylidar_param{
        float stop_width;
        float stop_height;
        float slow_width;
        float slow_height;
        float lidar_offset;
        float stop_back_left_width;
        float stop_back_right_width;
        float stop_back_height;
        float slow_back_height;
    };

    struct ylidar_eye_param{
        float xf1;
        float xf2;
        float xf3;
        float yl1;
        float yl2;
        float detect_angle1; //角度，小
        float detect_angle2; //角度，大
        float range_1;
        float range_2;
        float range_3;
    };

    class YoursLidarDetectHuman
    {
    private:
        void ScanCallback(const sensor_msgs::LaserScan& msg);
        void DetectClusters(const sensor_msgs::LaserScan& lidar, std::map<int, ycluster>& in_cluster);
        void ShowClusters(const std::map<int, ycluster> &in_cluster, double show_range);
        void ShowEyeClusters(const std::map<int, ycluster> &in_cluster, double show_range);
        
        void ShowEyeClustersPolar(const std::map<int, ycluster> &in_cluster, double show_range);
        
        void ClusterIsObstacle(const ypoint& in_point, std::vector<ycluster>& in_cluster);
        /**
         * @brief 分析获取的雷达数据
         * 除了需要输入的团块数据之外，还需要读取的雷达检测数据: ylidar_param 
         * 
         * @param in_cluters 
         */
        int AnalyseClusters(std::map<int, ycluster> &in_cluters);
        /* data */
        //std::shared_ptr<yours_log::YoursLog> log_;
        std::map<int, ycluster> clusters_;
        ros::NodeHandle n_;
        ros::Subscriber lidar_sub;
        ros::Publisher lidar_stop_pub;
        ros::Publisher eye_pub;
        sensor_msgs::LaserScan lidar_msg;
        double distance_threshold;
        ylidar_param lidar_detect_param_;
        ylidar_eye_param lidar_detect_eye_param_;

     public:
        //YoursLidarDetectHuman(ros::NodeHandle& n, std::shared_ptr<yours_log::YoursLog> log );
        YoursLidarDetectHuman(ros::NodeHandle& n);
        ~YoursLidarDetectHuman();
        void setStopParam(const double& widht, const double& height);
        void setBackStopParam(const double& left_widht,
                              const double& right_widht, const double& height);
        void run();
    };

}; // namespace yours_lidar_detect_human

#endif