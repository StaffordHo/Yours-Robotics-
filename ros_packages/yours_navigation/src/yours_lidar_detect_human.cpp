#include "../include/yours_navigation/yours_lidar_detect_human.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
using namespace yours_lidar_detect_human;

//YoursLidarDetectHuman::YoursLidarDetectHuman(ros::NodeHandle &n, std::shared_ptr<yours_log::YoursLog> log /* args */)
//:log_(log)
YoursLidarDetectHuman::YoursLidarDetectHuman(ros::NodeHandle &n /* args */)
:n_(n)
,distance_threshold(0.05)
{
    lidar_detect_param_.lidar_offset = n_.param("lidar_setup_offser", float(0.35)); //x轴方向
    lidar_detect_param_.stop_width = n_.param("stop_w", float(0.9)); //y 轴方向
    lidar_detect_param_.stop_height = n_.param("stop_h", float(0.9)); //x 轴方向
    lidar_detect_param_.slow_width = n_.param("slow_down_w", float(2.2));
    lidar_detect_param_.slow_height = n_.param("slow_down_h", float(2.2));
    lidar_detect_param_.stop_back_height = n_.param("stop_back_height", float(0.1));
    lidar_detect_param_.slow_back_height = n_.param("slow_back_height", float(0.10));
    lidar_detect_param_.stop_back_left_width = n_.param("stop_back_left_width", float(0.6));
    lidar_detect_param_.stop_back_right_width = n_.param("stop_back_right_width", float(0.6));

    lidar_detect_eye_param_.xf1 = 1.0;
    lidar_detect_eye_param_.xf2 = 2.0;
    lidar_detect_eye_param_.xf3 = 3.5;
    lidar_detect_eye_param_.yl1 = 0.5;
    lidar_detect_eye_param_.yl2 = 2.0;
    lidar_detect_eye_param_.detect_angle1 = n_.param("eye_polar_angle_1", float(20.0));
    lidar_detect_eye_param_.detect_angle2 = n_.param("eye_polar_angle_2", float(60.0));
    lidar_detect_eye_param_.range_1 = n_.param("eye_polar_range_1", float(1.0));
    lidar_detect_eye_param_.range_2 = n_.param("eye_polar_range_2", float(2.0));
    lidar_detect_eye_param_.range_3 = n_.param("eye_polar_range_3", float(3.5));
    lidar_sub = n_.subscribe("/scan_filter", 1, &YoursLidarDetectHuman::ScanCallback, this);
    lidar_stop_pub = n_.advertise<std_msgs::UInt8>("/yours_laser_detect_stop_slow_down", 10);
    eye_pub = n_.advertise<std_msgs::String>("/yours_base/face_position", 10);
}

YoursLidarDetectHuman::~YoursLidarDetectHuman()
{
}

void YoursLidarDetectHuman::ScanCallback(const sensor_msgs::LaserScan &msg)
{
    lidar_msg = msg;
}
void YoursLidarDetectHuman::DetectClusters(const sensor_msgs::LaserScan &lidar, std::map<int, ycluster> &in_cluster)
{
    ycluster cluster;
    int map_id = 0;

    for (int i = 0; i < lidar.ranges.size();i++)
    {
        // 前向 mini_lidar 上下装反，扫描方向相反导致左右镜像；角度取反修正(只翻左右，不动前后)
        double angle = -(lidar.angle_min + (i * lidar.angle_increment));
        //ROS_INFO_STREAM("angle =  " << angle);
        double range = lidar.ranges[i];
        ypoint xypoint;
        if(!xypoint.setXYPoint(angle, range)){
            //ROS_INFO_STREAM("point is invalue");
            continue;
        }
        if(cluster.cluster_.empty()){
            cluster.cluster_.push_back(xypoint);
            //ROS_INFO_STREAM("scan start");
            continue;
        }
        //计算当前点与上一个点距离
        double dist = xypoint.getDistance(cluster.cluster_[cluster.cluster_.size() - 1]);
        //ROS_INFO_STREAM("dist = " << dist);
        if(dist < distance_threshold){
            cluster.cluster_.push_back(xypoint);

        }else{
            if (cluster.cluster_.size() > 10)
            {
                //存储当前cluster
                in_cluster[map_id] = cluster;
                //更新数据
                map_id++;
            }
            cluster.clear();
            cluster.cluster_.push_back(xypoint);
        }
    }
    //添加最后一组数据
    if(cluster.cluster_.size() > 5){
        in_cluster[map_id] = cluster;
    }
}

void drawDetectRectangle(cv::Mat& show_mat, const ylidar_param& param, const int& show_range){
    if(!show_mat.empty()){
    //x轴
    cv::line(show_mat, cv::Point(0, show_mat.rows / 2), cv::Point(show_mat.cols, show_mat.rows / 2), cv::Scalar(0, 0, 200));
    //y轴
    cv::line(show_mat, cv::Point(show_mat.cols/2, 0), cv::Point(show_mat.cols/2, show_mat.rows), cv::Scalar(0, 200, 0));
    //停车区域

    //雷达坐标系
    int half_width = (show_mat.cols / 2);
    int half_height = (show_mat.rows / 2);
    ///下横线
    ////开始点
    float xs_lidar = -0.45; //-(param.stop_height / 2.0);
    float ys_lidar = param.stop_width/ 2.0;
    ////结束点
    float xe_lidar = param.stop_height;
    float ye_lidar = param.stop_width / 2.0;
    //图像坐标系
    cv::Point p1;
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    cv::Point p2;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(0, 0, 220)); //red
    ///上横线
    ////开始点
    xs_lidar = -0.45; //-(param.stop_height / 2.0);
    ys_lidar = -param.stop_width/ 2.0;
    ////结束点
    xe_lidar = param.stop_height; //param.stop_height / 2.0;
    ye_lidar = -param.stop_width / 2.0;
    //图像坐标系
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(0, 0, 220)); //red
    ///左竖线
    ////开始点
    xs_lidar = -0.45; //-(param.stop_height / 2.0);
    ys_lidar = -param.stop_width/ 2.0;
    ////结束点
    xe_lidar = -0.45; //-(param.stop_height / 2.0);
    ye_lidar = param.stop_width / 2.0;
    //图像坐标系
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(0, 0, 220)); //red
    ///右竖线
    ////开始点
    xs_lidar = param.stop_height; //(param.stop_height / 2.0);
    ys_lidar = -param.stop_width / 2.0;
    ////结束点
    xe_lidar = param.stop_height; //(param.stop_height / 2.0);
    ye_lidar = param.stop_width / 2.0;
    //图像坐标系
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(0, 0, 220)); //red
    //慢速区域
    ///下横线
    ////开始点
    xs_lidar = -0.5; //-(param.slow_height / 2.0);
    ys_lidar = param.slow_width / 2.0;
    ////结束点
    xe_lidar = param.slow_height; //(param.slow_height / 2.0);
    ye_lidar = param.slow_width / 2.0;
    //图像坐标系
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(255, 120, 0)); //blue
    ///上横线
    ////开始点
    xs_lidar = -0.5; //-(param.slow_height / 2.0);
    ys_lidar = -param.slow_width / 2.0;
    ////结束点
    xe_lidar = param.slow_height;//(param.slow_height / 2.0);
    ye_lidar = -param.slow_width / 2.0;
    //图像坐标系
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(255, 120, 0));
    ///左竖线
    ////开始点
    xs_lidar = param.slow_height; //(param.slow_height / 2.0);
    ys_lidar = -param.slow_width / 2.0;
    ////结束点
    xe_lidar = param.slow_height;//(param.slow_height / 2.0);
    ye_lidar = param.slow_width / 2.0;
    //图像坐标系
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(255, 120, 0));
    ///右竖线
    ////开始点
    xs_lidar = -0.5; //-(param.slow_height / 2.0);
    ys_lidar = -param.slow_width / 2.0;
    ////结束点
    xe_lidar = -0.5; //-(param.slow_height / 2.0);
    ye_lidar = param.slow_width / 2.0;
    //图像坐标系
    p1.x = ((xs_lidar / show_range) * half_width) + half_width;
    p1.y = ((ys_lidar / show_range) * half_height) + half_height;
    p2.x = ((xe_lidar / show_range) * half_width) + half_width;
    p2.y = ((ye_lidar / show_range) * half_height) + half_height;
    cv::line(show_mat, p1, p2, cv::Scalar(255, 120, 0));
    }
}

cv::Point2f LidarPoint2MatPoint(const cv::Point2f& lp, const float& show_range, const float& mat_width, const float& mat_height) { cv::Point2f mat_point;
    float half_width = mat_width / 2.0;
    float half_height = mat_height / 2.0;
    mat_point.x = ((lp.x / show_range) * half_width) + half_width;
    mat_point.y = ((lp.y / show_range) * half_height) + half_height;

    return mat_point;
}

void drawEyeDetectRectangle(cv::Mat& show_mat, const ylidar_eye_param& param, const int& show_range) {
    // x<0在后面  y > 0 在车左面，y < 0在车右面 2021年6月8日结论

    if (!show_mat.empty()) {
        // x轴 red
        cv::line(show_mat, cv::Point(0, show_mat.rows / 2), cv::Point(show_mat.cols, show_mat.rows / 2), cv::Scalar(0, 0, 200));
        // y轴 green
        cv::line(show_mat, cv::Point(show_mat.cols / 2, 0), cv::Point(show_mat.cols / 2, show_mat.rows), cv::Scalar(0, 200, 0));

        cv::Point2f l_p = LidarPoint2MatPoint(cv::Point2f(1.0, 2.0), show_range, show_mat.cols, show_mat.rows);
        cv::Point2f r_p = LidarPoint2MatPoint(cv::Point2f(1.0, -2.0), show_range, show_mat.cols, show_mat.rows);
        cv::line(show_mat, l_p, r_p, cv::Scalar(200, 0, 0));

        l_p = LidarPoint2MatPoint(cv::Point2f(1.0, 2.0), show_range, show_mat.cols, show_mat.rows);
        r_p = LidarPoint2MatPoint(cv::Point2f(2.0, 2.0), show_range, show_mat.cols, show_mat.rows);
        cv::line(show_mat, l_p, r_p, cv::Scalar(200, 0, 0));
    
        l_p = LidarPoint2MatPoint(cv::Point2f(3.5, 2.0), show_range, show_mat.cols, show_mat.rows);
        r_p = LidarPoint2MatPoint(cv::Point2f(3.5, -2.0), show_range, show_mat.cols, show_mat.rows);
        cv::line(show_mat, l_p, r_p, cv::Scalar(200, 0, 0));
    }
}

void FilterEyeClusters(std::map<int, ycluster>& in_cluster, const ylidar_eye_param& eye_param, const bool& is_polar = false) {
    for (auto it : in_cluster) {
        ycluster cluster = it.second;
        //每个雷达点blob
        int filter_cnt = 0;
        float fullX = 0;
        if (is_polar) {
            float radian = (eye_param.detect_angle2 / 180.0) * M_PI;
            for (auto cit : cluster.cluster_) {
                if ((cit.range_ < eye_param.range_3) && (cit.range_ > 0.0) && (cit.angle_ < radian/2.0) && (cit.angle_ > -radian/2.0) ) {
                    filter_cnt++;
                    fullX = cit.x + fullX;
                }
            }

        } else {
            for (auto cit : cluster.cluster_) {
                if ((cit.x > 1.0) && (cit.x < 3.5) && (cit.y < 2.0) && (cit.y > -2.0)) {
                    filter_cnt++;
                    fullX = cit.x + fullX;
                }
            }
        }

        if (filter_cnt > 10) {
            in_cluster[it.first].isEyeCandidacy = true;
            // it.second.isEyeCandidacy = true;
            in_cluster[it.first].x2lidar = (fullX / filter_cnt);
        }
    }
}

void FindNearlyClusters(std::map<int, ycluster>& in_cluster) {
    std::pair<int, float> near_param;
    near_param.first = -1;
    near_param.second = 999.0;
    for (auto it : in_cluster) {
        if (it.second.isEyeCandidacy) {
            if (it.second.x2lidar < near_param.second) {
                near_param.second = it.second.x2lidar;
                near_param.first = it.first;
            }
        }
    }
    if (near_param.first > 0) {
        in_cluster[near_param.first].isNearestRobot4Eye = true;
    }
}


void YoursLidarDetectHuman::ShowEyeClusters(const std::map<int, ycluster> &in_cluster, double show_range){
    int color_cnt = 2;
    cv::Mat showImage(1000, 1000, CV_8UC3, cv::Scalar::all(0));
    int half_width = (showImage.cols / 2);
    int half_height = (showImage.rows / 2);

    if (!in_cluster.empty()) {
        // ROS_INFO_STREAM("map size =  " << in_cluster.size());
        for (auto it : in_cluster) {
            ycluster cluster = it.second;
            cv::Scalar color;

            color = cv::Scalar(200, 200, 200);
            if (cluster.isEyeCandidacy) {
                color = cv::Scalar(200, 0, 0);
            }
            if(cluster.isNearestRobot4Eye){
            std::cout << "find eye : " << it.first << std::endl;
                color = cv::Scalar(10, 0, 200);
            }

#if 1
            for (auto cit : cluster.cluster_) {
                cv::Point2d cvpoint;
                // cvpoint.x = (((cit.x - lidar_detect_param_.lidar_offset) / show_range) * (showImage.cols/2)  + (showImage.cols/2));
                cvpoint.x = (((cit.x - 0) / show_range) * (showImage.cols / 2) + (showImage.cols / 2));
                cvpoint.y = ((cit.y / show_range) * (showImage.rows / 2) + (showImage.rows / 2));

                cv::drawMarker(showImage, cvpoint, color, cv::MARKER_SQUARE, 3);
            }
#endif

            //显示blob区域

            int end_point = cluster.cluster_.size() - 1;
            cv::Point cvpoint;
            //cvpoint.x = (((cluster.cluster_[0].x - lidar_detect_param_.lidar_offset) / show_range) * half_width + 500.0);
            cvpoint.x = (((cluster.cluster_[0].x - 0) / show_range) * half_width + 500.0);
            cvpoint.y = ((cluster.cluster_[0].y / show_range) * 500.0 + 500.0);

            cv::Point cvpoint2;
            //cvpoint2.x = (((cluster.cluster_[end_point].x - lidar_detect_param_.lidar_offset) / show_range) * 500.0 + 500.0);
            cvpoint2.x = (((cluster.cluster_[end_point].x - 0) / show_range) * 500.0 + 500.0);
            cvpoint2.y = ((cluster.cluster_[end_point].y / show_range) * 500.0 + 500.0);
            cv::line(showImage, cvpoint, cvpoint2, cv::Scalar(150, 150, 150), 1
                     // color, 3
            );

            // color_cnt++;
            if (color_cnt == 3) {
                color_cnt = 0;
            }
        }
    }

    drawEyeDetectRectangle(showImage, lidar_detect_eye_param_, show_range);
    cv::flip(showImage, showImage, 1);
    cv::imshow("lidar_eye", showImage);
    cv::waitKey(10);
}

void drawPolarLine(cv::Mat& show_mat, float show_range, float angle, float range, const cv::Scalar& color) { 
    ypoint pp1;
    pp1.setXYPoint(angle, range);
    cv::Point2f lp1(0, 0);
    cv::Point2f lp2(pp1.x, pp1.y);
    lp1 = LidarPoint2MatPoint(lp1, show_range, show_mat.cols, show_mat.rows);
    lp2 = LidarPoint2MatPoint(lp2, show_range, show_mat.cols, show_mat.rows);
    cv::line(show_mat, lp1, lp2, color);
}

void drawPolarLineConnect(cv::Mat&show_mat, float show_range, const cv::Scalar& color, std::pair<float, float> _pp1, std::pair<float, float> _pp2) { ypoint pp1;
    ypoint pp2;
    pp1.setXYPoint(_pp1.first, _pp1.second);
    pp2.setXYPoint(_pp2.first, _pp2.second);
    cv::Point2f lp1(pp1.x, pp1.y);
    cv::Point2f lp2(pp2.x, pp2.y);
    lp1 = LidarPoint2MatPoint(lp1, show_range, show_mat.cols, show_mat.rows);
    lp2 = LidarPoint2MatPoint(lp2, show_range, show_mat.cols, show_mat.rows);
    cv::line(show_mat, lp1, lp2, color);
}

void drawEyeDetectPolarArea(cv::Mat & show_mat, const ylidar_eye_param& param, float show_range) {
        if (!show_mat.empty()) {
    //x轴
    cv::line(show_mat, cv::Point(0, show_mat.rows / 2), cv::Point(show_mat.cols, show_mat.rows / 2), cv::Scalar(0, 0, 200));
    //y轴
    cv::line(show_mat, cv::Point(show_mat.cols/2, 0), cv::Point(show_mat.cols/2, show_mat.rows), cv::Scalar(0, 200, 0));
            //float a1 = (param.detect_angle / 3.0) / 2.0;
            //float a2 = param.detect_angle / 4.0;
            float a3 = M_PI*((param.detect_angle2 / 2.0)/180.0)  ;
            drawPolarLine(show_mat, show_range, a3 , param.range_3,  cv::Scalar(0,200,0));
            drawPolarLine(show_mat, show_range, -a3 , param.range_3,   cv::Scalar(0,200,0));
            drawPolarLine(show_mat, show_range, a3 , param.range_2,  cv::Scalar(200,0,0));
            drawPolarLine(show_mat, show_range, -a3 , param.range_2,   cv::Scalar(200,0,0));
            drawPolarLine(show_mat, show_range, a3 , param.range_1, cv::Scalar(0,0,200));
            drawPolarLine(show_mat, show_range, -a3 , param.range_1, cv::Scalar(0,0,200));
            float a2 = (param.detect_angle1 / 2.0) * (M_PI/ 180.0);
            drawPolarLine(show_mat, show_range,  a2 , param.range_3, cv::Scalar(0, 200, 0));
            drawPolarLine(show_mat, show_range,  -a2 , param.range_3,   cv::Scalar(0,200,0));
            drawPolarLine(show_mat, show_range,  a2 , param.range_2,  cv::Scalar(200,0,0));
            drawPolarLine(show_mat, show_range,  -a2 , param.range_2,   cv::Scalar(200,0,0));
            drawPolarLine(show_mat, show_range,  a2 , param.range_1, cv::Scalar(0,0,200));
            drawPolarLine(show_mat, show_range,  -a2 , param.range_1, cv::Scalar(0,0,200));

            std::pair<float, float> pp1;
            std::pair<float, float> pp2;
            //区域1
            pp1.first = a3;
            pp1.second = param.range_1;
            pp2.first = a2;
            pp2.second = param.range_1;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(0, 0, 200), pp1, pp2);
            pp1.first = -a2;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(0, 0, 200), pp1, pp2);
            pp1.first = -a2;
            pp2.first = -a3;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(0, 0, 200), pp1, pp2);
            //区域2 
            pp1.first = a3;
            pp1.second = param.range_2;
            pp2.first = a2;
            pp2.second = param.range_2;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(200, 0, 0), pp1, pp2);
            pp1.first = -a2;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(200, 0, 0), pp1, pp2);
            pp1.first = -a2;
            pp2.first = -a3;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(200, 0, 0), pp1, pp2);
            //区域3
            pp1.first = a3;
            pp1.second = param.range_3;
            pp2.first = a2;
            pp2.second = param.range_3;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(0, 200, 0), pp1, pp2);
            pp1.first = -a2;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(0, 200, 0), pp1, pp2);
            pp1.first = -a2;
            pp2.first = -a3;
            drawPolarLineConnect(show_mat, show_range, cv::Scalar(0, 200, 0), pp1, pp2);
        
        
        
        }
}

int AnalyseEyeClusters(const std::map<int, ycluster>& in_cluter, const ylidar_eye_param& _param){
            int rr = -1;
    for(auto it:in_cluter){
        if(it.second.isNearestRobot4Eye){
            int a1_cnt = 0;
            int a2_cnt = 0;
            int a3_cnt = 0;
            int a4_cnt = 0;
            int a5_cnt = 0;
            int a6_cnt = 0;
            float radian1 = (M_PI * _param.detect_angle1 / 180.0) / 2.0 ;
            float radian2 = (M_PI * _param.detect_angle2 / 180.0) / 2.0;
            for (auto cit : it.second.cluster_) {
                if((cit.angle_<radian2) && (cit.angle_> radian1)){
                    if ((cit.range_ > _param.range_1) && (cit.range_ < _param.range_2)) {
                        a3_cnt++;
                    } else if ((cit.range_ > _param.range_2) && (cit.range_ < _param.range_3)) {
                        a6_cnt++;
                    }
                }else if((cit.angle_ > -radian1) && (cit.angle_< radian1)){
                    if ((cit.range_ > _param.range_1) && (cit.range_ < _param.range_2)) {
                        a2_cnt++;
                    } else if ((cit.range_ > _param.range_2) && (cit.range_ < _param.range_3)) {
                        a5_cnt++;
                    }
//                    a2_cnt++;
                } else if ((cit.angle_ > -radian2) && (cit.angle_ < -radian1)) {
                    if ((cit.range_ > _param.range_1) && (cit.range_ < _param.range_2)) {
                        a1_cnt++;
                    } else if ((cit.range_ > _param.range_2) && (cit.range_ < _param.range_3)) {
                        a4_cnt++;
                    }
//                    a1_cnt++;
                }
            }
            int max_cnt = -1;
            if (a1_cnt > a2_cnt) {
                rr = 1;
                max_cnt = a1_cnt;
            }else{
                rr = 2;
                max_cnt = a2_cnt;
            }

            if(a3_cnt > max_cnt){
                max_cnt = a3_cnt;
                rr = 3;
            }

            if(a4_cnt > max_cnt){
                max_cnt = a4_cnt;
                rr = 4;
            }
            if(a5_cnt > max_cnt){
                max_cnt = a5_cnt;
                rr = 5;
            }
            if(a6_cnt > max_cnt){
                max_cnt = a6_cnt;
                rr = 6;
            }

//            if(max_cnt > 0){
//                std::cout << " Area : " << rr << " cnt: " << max_cnt << std::endl;
//            }
        }


    }

    return rr;
}

void YoursLidarDetectHuman::ShowEyeClustersPolar(const std::map<int, ycluster>& in_cluster, double show_range) {
    int color_cnt = 2;
    cv::Mat showImage(1000, 1000, CV_8UC3, cv::Scalar::all(0));
    int half_width = (showImage.cols / 2);
    int half_height = (showImage.rows / 2);
    if (!in_cluster.empty()) {
        // ROS_INFO_STREAM("map size =  " << in_cluster.size());
        for (auto it : in_cluster) {
            ycluster cluster = it.second;
            cv::Scalar color;

            color = cv::Scalar(200, 200, 200);
            if (cluster.isEyeCandidacy) {
                color = cv::Scalar(200, 0, 0);
            }
            if (cluster.isNearestRobot4Eye) {
//                std::cout << "find eye : " << it.first << std::endl;
                color = cv::Scalar(10, 0, 200);
            }

#if 1
            for (auto cit : cluster.cluster_) {
                cv::Point2d cvpoint;
                // cvpoint.x = (((cit.x - lidar_detect_param_.lidar_offset) / show_range) * (showImage.cols/2)  + (showImage.cols/2));
                cvpoint.x = (((cit.x - 0) / show_range) * (showImage.cols / 2) + (showImage.cols / 2));
                cvpoint.y = ((cit.y / show_range) * (showImage.rows / 2) + (showImage.rows / 2));

                cv::drawMarker(showImage, cvpoint, color, cv::MARKER_SQUARE, 3);
            }
#endif

            //显示blob区域

            int end_point = cluster.cluster_.size() - 1;
            cv::Point cvpoint;
            // cvpoint.x = (((cluster.cluster_[0].x - lidar_detect_param_.lidar_offset) / show_range) * half_width + 500.0);
            cvpoint.x = (((cluster.cluster_[0].x - 0) / show_range) * half_width + 500.0);
            cvpoint.y = ((cluster.cluster_[0].y / show_range) * 500.0 + 500.0);

            cv::Point cvpoint2;
            // cvpoint2.x = (((cluster.cluster_[end_point].x - lidar_detect_param_.lidar_offset) / show_range) * 500.0 + 500.0);
            cvpoint2.x = (((cluster.cluster_[end_point].x - 0) / show_range) * 500.0 + 500.0);
            cvpoint2.y = ((cluster.cluster_[end_point].y / show_range) * 500.0 + 500.0);
            cv::line(showImage, cvpoint, cvpoint2, cv::Scalar(150, 150, 150), 1
                     // color, 3
            );

            // color_cnt++;
            if (color_cnt == 3) {
                color_cnt = 0;
            }
        }
    }

    drawEyeDetectPolarArea(showImage, lidar_detect_eye_param_, show_range);
    cv::flip(showImage, showImage, 1);
    cv::imshow("lidar_eye_polar", showImage);
    cv::waitKey(10);
}

void YoursLidarDetectHuman::ShowClusters(const std::map<int, ycluster> &in_cluster, double show_range)
{
    int color_cnt = 2;
    cv::Mat showImage(1000, 1000, CV_8UC3, cv::Scalar::all(0));
    int half_width = (showImage.cols / 2);
    int half_height = (showImage.rows / 2);
	
	//ROS_INFO_STREAM( lidar_detect_param_.stop_width << " "  << lidar_detect_param_.stop_height << " " <<lidar_detect_param_.slow_width << " " << lidar_detect_param_.slow_height  );

    if (!in_cluster.empty())
    {
        //ROS_INFO_STREAM("map size =  " << in_cluster.size());
        for (auto it : in_cluster)
        {
            ycluster cluster = it.second;
            cv::Scalar color;
            for (auto cit : cluster.cluster_)
            {
                cv::Point2d cvpoint;
                cvpoint.x = (((cit.x - lidar_detect_param_.lidar_offset) / show_range) * (showImage.cols/2)  + (showImage.cols/2));
                cvpoint.y = ((cit.y / show_range) * (showImage.rows/2) + (showImage.rows/2));

                /*
                if (color_cnt == 0)
                {
                    color = cv::Scalar(255, 0, 0);
                }else if(color_cnt == 1){
                    color = cv::Scalar(0, 255, 0);
                }else if(color_cnt == 2){
                    color = cv::Scalar(0,0,255);

                }*/
                color = cv::Scalar(200, 200, 200);
                if(it.second.isStop){
                    color = cv::Scalar(0, 0, 255);
                }
                if(it.second.isSlow){
                    color = cv::Scalar(255, 50, 0);

                }


                cv::drawMarker(showImage, cvpoint, color, cv::MARKER_SQUARE, 3);
            }
            int end_point = cluster.cluster_.size() - 1;


            cv::Point cvpoint;
            cvpoint.x = (((cluster.cluster_[0].x - lidar_detect_param_.lidar_offset) / show_range) * half_width + 500.0);
            cvpoint.y = ((cluster.cluster_[0].y / show_range) * 500.0 + 500.0);

            cv::Point cvpoint2;
            cvpoint2.x = (((cluster.cluster_[end_point].x - lidar_detect_param_.lidar_offset)/ show_range) * 500.0 + 500.0);
            cvpoint2.y = ((cluster.cluster_[end_point].y / show_range) * 500.0 + 500.0);
            cv::line(showImage,
                     cvpoint,
                     cvpoint2,
                     cv::Scalar(150, 150, 150),1
                    //color, 3
            );



            //color_cnt++;
            if(color_cnt == 3){
                color_cnt = 0;
            }
        }
    }
    drawDetectRectangle(showImage, lidar_detect_param_, show_range);
    cv::flip(showImage, showImage, 1);
    cv::imshow("lidar", showImage);
    cv::waitKey(10);
}

int YoursLidarDetectHuman::AnalyseClusters(
    std::map<int, ycluster>& in_cluters) {
    int result = 0;

    for (auto it_map : in_cluters) {
        int stop_cnt = 0;
        int slow_cnt = 0;
        int back_stop_cnt = 0;
        int back_slow_cnt = 0;
        for (auto it_point : it_map.second.cluster_) {
            if (((it_point.x - lidar_detect_param_.lidar_offset) > -0.35)
                && ((it_point.x - lidar_detect_param_.lidar_offset)
                    < (lidar_detect_param_.stop_height))
                && (fabs(it_point.y)
                    < (lidar_detect_param_.stop_width / 2.0))) {
                stop_cnt++;
            }

            // if ((fabs(it_point.x - lidar_detect_param_.lidar_offset) <
            // (lidar_detect_param_.slow_height/2.0)) && (fabs(it_point.y) <
            // (lidar_detect_param_.slow_width/2.0)))
            if (((it_point.x - lidar_detect_param_.lidar_offset) > -0.5)
                && ((it_point.x - lidar_detect_param_.lidar_offset)
                    < (lidar_detect_param_.slow_height))
                && (fabs(it_point.y)
                    < (lidar_detect_param_.slow_width / 2.0))) {
                slow_cnt++;
                // if (it_point.y < 0) {
                //    std::cout << "y < 0" << std::endl;
                //}
            }

            // x<0在后面  y > 0 在车左面，y < 0在车右面 2021年6月8日结论
            if (it_point.y > -fabs(lidar_detect_param_.stop_back_right_width)
                && (it_point.y
                    < fabs(lidar_detect_param_.stop_back_left_width))) {
                if ((it_point.x < 0)
                    && (it_point.x > -fabs(
                            lidar_detect_param_.stop_back_height))) {  // x方向限定，首先点要在雷达后面，且在设定的宽度以内
                    back_stop_cnt++;
                            }
                if ((it_point.x < 0)
                    && (it_point.x > -fabs(
                            lidar_detect_param_.slow_back_height ))) {  // x方向限定，首先点要在雷达后面，且在设定的宽度以内
                
                    back_slow_cnt++;
                }
            }
        }
        if (stop_cnt > 5) {
            // ROS_INFO_STREAM(" stop lidar :  " << it_map.first);
            in_cluters[it_map.first].isStop = true;

            result = (result | 2);
        } else if (slow_cnt > 5) {
            // ROS_INFO_STREAM(" slow lidar :  " << it_map.first);
            in_cluters[it_map.first].isSlow = true;
            result = (result | 1);
        }

        if(back_stop_cnt > 5){
            in_cluters[it_map.first].isNearlyWall = true;
            result = (result | 4);
            //std::cout << " have wall. wall point: " <<  back_stop_cnt << std::endl;
        }else if(back_slow_cnt > 5){
            in_cluters[it_map.first].isNearlyWall = true;
            result = (result | 8); //bit 3
            //std::cout << " near wall. wall point: " <<  back_slow_cnt << std::endl;
        }
    }
    return result;
}

void YoursLidarDetectHuman::setStopParam(const double& wight, const double& height){
    lidar_detect_param_.stop_height = height;
    lidar_detect_param_.stop_width = wight;
}

void YoursLidarDetectHuman::setBackStopParam(const double& left_widht, const double& right_widht, const double& height){
    lidar_detect_param_.stop_back_left_width = left_widht;
    lidar_detect_param_.stop_back_right_width = right_widht;
    lidar_detect_param_.stop_back_height = height;
}

void YoursLidarDetectHuman::run(){
    //后面要做多线程同步
    sensor_msgs::LaserScan lidar = lidar_msg;
    std_msgs::UInt8 stop;
    int eye_look_at = -1;
    if(!lidar.ranges.empty()){
        DetectClusters(lidar, clusters_);
        stop.data = AnalyseClusters(clusters_);
        //ShowClusters(clusters_, 5);
        FilterEyeClusters(clusters_, lidar_detect_eye_param_, true);
        FindNearlyClusters(clusters_);
        //ShowEyeClusters(clusters_, 5);
        eye_look_at = AnalyseEyeClusters(clusters_, lidar_detect_eye_param_);
        //ShowEyeClustersPolar(clusters_, 5);
        lidar.ranges.clear();
        clusters_.clear();
    }
    std_msgs::String eye_pose_msg;
    eye_pose_msg.data = "center";
    if (eye_look_at < 0) {
        eye_pose_msg.data = "center";
    } else if (eye_look_at == 1) {
        eye_pose_msg.data = "leftup";
    } else if (eye_look_at == 2) {
        eye_pose_msg.data = "up";
    } else if (eye_look_at == 3) {
        eye_pose_msg.data = "rightup";
    } else if (eye_look_at == 4) {
        eye_pose_msg.data = "left";
    } else if (eye_look_at == 6) {
        eye_pose_msg.data = "right";
    } else{
        eye_pose_msg.data = "center";
    }
    eye_pub.publish(eye_pose_msg);

    // bit 0 = slow  bit 1 = stop bit 2 = nearly_wall
    //stop.data = 0;
    lidar_stop_pub.publish(stop);
}
