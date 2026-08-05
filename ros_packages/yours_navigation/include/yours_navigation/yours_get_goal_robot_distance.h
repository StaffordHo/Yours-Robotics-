#ifndef YOURS_GET_GOAL_ROBOT_DISTANCE_H_
#define YOURS_GET_GOAL_ROBOT_DISTANCE_H_
#include <ros/ros.h>
#include <yours_robot_tools/yours_log.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <std_msgs/Float64MultiArray.h>
#include "robot.h"
class YoursGetGoalRobotDistance {
 private:
    /* data */
    std::shared_ptr<yours_log::YoursLog> log_;
    std::map<YoursPath, std::vector<YoursNavPoint>> robot_path_;
    std::vector<YoursNavPoint> sell_path_;
    double on_loop_distance;
    int robot_cnt;
    //单车
    //YoursNavPoint remote_nav_point;
    //多车
    std::vector<YoursNavPoint> remote_nav_point;

    ros::Subscriber remote_pose_sub;
    ros::Subscriber remote_robot_cnt_sub;
    bool is_get_remote;
    double stop_distance_;
    /*
    void RemoteCallback(const nav_msgs::Odometry& pose) {
        tf::Quaternion q(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
        double yaw = tf::getYaw(q);
        remote_nav_point.point.x = pose.pose.pose.position.x;
        remote_nav_point.point.y = pose.pose.pose.position.y;
        remote_nav_point.point.yaw = yaw;
        is_get_remote = true;
        //ROS_INFO_STREAM("get remote pose");
    }
    */
    void RemoteCallback(const std_msgs::Float64MultiArray& pose) {
        if((pose.data.size() % 3) == 0){
            remote_nav_point.clear();
            for (int i = 0; i < (pose.data.size() / 3); i ++){
                YoursNavPoint npoint(pose.data[i*3 + 1], pose.data[i*3 + 2], pose.data[i*3]);
                remote_nav_point.push_back(npoint);
            }
            is_get_remote = true;
            //加上自己
            robot_cnt = (pose.data.size() / 3) + 1;
        }
    }


    void RobotCntCallback(const std_msgs::Int32& msg) { robot_cnt = msg.data;
        log_->info("robot cnt :" + std::to_string(robot_cnt));
        
    }

    double Distance(const YoursNavPoint& p1,  const YoursNavPoint& p2){
		double x1 = p1.point.x, y1 = p1.point.y;
		double x2 = p2.point.x, y2 = p2.point.y;
		double dis = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		return dis;
	}

    //正向循环查找，
    double DistanceFromVectorID1ToVectorID2(const std::vector<YoursNavPoint>& path, int id_1, int id_2){
        if(id_1 == id_2){
            return 0.0;
        }
        int i = 0;
        int cnt = id_1;
        double all_distance = 0.0;
        for (i = 0; i < (path.size() + 1); i++) {
            YoursNavPoint p1 = path[cnt];
            YoursNavPoint p2;
            //p1是最后一个点，所以p2应该是第一个点
            if(cnt == (path.size() - 1)){
                p2 = path[0];
            }else{
                p2 = path[cnt + 1];
            }
            all_distance = Distance(p1, p2) + all_distance;
            
            //下标变换
            cnt++;
            if(cnt == path.size()){
                cnt = 0;
            }
            //下标变为id_2代表已经计算完成，退出
            if(cnt == id_2){
                break;
            }
        }

        if(i == path.size()){
            log_->info("i == path.size(), loop overflow, error");
            return -1.0;
        }

        return all_distance;
    }

    //查找path中的最近点
    int FindNearestPoint(const std::vector<YoursNavPoint>& path, const YoursNavPoint& pose) {
        double min_distance = 999.0;
        int id_temp = -1;
        for (int i = 0; i < path.size(); i++) {
            YoursNavPoint vp = path[i];
            double dis = Distance(vp, pose);
            if (dis < min_distance) {
                min_distance = dis;
                id_temp = i;
            }
        }
        return id_temp;
    };


    /**
     * @brief Get the Pass And Goal Point Vector I D object
     * 根据当前pose计算经过点和目标点
     * 默认小车从小的点到大的点行走，若在最后一个点，则接着第一个点。所以算法就是找到两个最近点，编号小的是经过的，编号大的是目标点
     * 
     * @param path 
     * @param pose 
     * @param pass_id 
     * @param goal_id 
     * @return true 
     * @return false 
     */
    bool getPassAndGoalPointVectorID(const std::vector<YoursNavPoint>& path, const YoursNavPoint& pose, int& pass_id, int& goal_id) { 
        int nearest_point_id = FindNearestPoint(path, pose);
        int _last = nearest_point_id - 1;
        int _next = nearest_point_id + 1;
        if(_last < 0){
            _last = path.size() - 1;
        }
        if(_next > (path.size() - 1)){
            _next = 0;
        }
        //判断小车在谁之间，判断依据：小车到邻近点的距离要小于邻近点到最近点的距离
        pass_id = -1;
        goal_id = -1;
        double dis_pose2last = Distance(pose, path[_last]);
        //推断经过点和目标点
        if(dis_pose2last < Distance(path[nearest_point_id], path[_last])){
            //机器人到last点的距离小于 last点和最近点间距离，所以机器人位于last点和最近点之间，且经过点是last，目标点是goal
            pass_id = _last;
            goal_id = nearest_point_id;
        }else{
            pass_id = nearest_point_id;
            goal_id = _next;
        }

        dis_pose2last = Distance(pose, path[pass_id]);
        double dis_pose2goal = Distance(pose, path[goal_id]);
        if( (dis_pose2goal + dis_pose2last) >  (Distance(path[pass_id], path[goal_id])*1.3) ){
            return false;
        }

        return true;
        //比最近点小的编号距离小车更近，所以小车位置 last---pose---nearest
    };


    /**
     * @brief Get the Distance From To object
     * 获取p1到p2位置的距离
     * p1->p2, p1头部到p2尾部
     * @param p1 本机器人位置
     * @param p2 另外一个机器人的位置
     * @return double 返回距离,单位米
     */
    bool getDistanceFromRobot1ToRobot2(YoursNavPoint p1, YoursNavPoint p2, double& dis) { 
        int robot_1_pass_point_id = -1;
        int robot_1_goal_point_id = -1;
        int robot_2_pass_point_id = -1;
        int robot_2_goal_point_id = -1;
        if(!getPassAndGoalPointVectorID(sell_path_, p1, robot_1_pass_point_id, robot_1_goal_point_id)){
            return false;
        }else{
        }
        if(!getPassAndGoalPointVectorID(sell_path_, p2, robot_2_pass_point_id, robot_2_goal_point_id)){
            return false;
        }else{
            //ROS_INFO_STREAM("robot 1 pass_id " << robot_1_pass_point_id << " goal_id " << robot_1_goal_point_id);
            //ROS_INFO_STREAM("robot 2 pass_id " << robot_2_pass_point_id << " goal_id " << robot_2_goal_point_id);

        }
        //p1在后 p2在前，所以计算方法为 p1的goal点到p2的pass点距离 加上 p1到到goal点和p2到pass点的距离
        double p1_to_p2;
        if ((robot_1_goal_point_id == robot_2_goal_point_id) && (robot_1_pass_point_id == robot_2_pass_point_id)) {
            //特殊情况 1和2在同路径上
            
            //特殊情况  p1在前 p1 距离goal近，这样p1要加速

            if( Distance(p1, sell_path_[robot_1_goal_point_id]) > Distance(p2, sell_path_[robot_2_goal_point_id]) ){
                //p1 目标点的距离 大于 p2到目标点的距离，且p1追p2，这说明 p1在p2之后，且是同一个路径，这样p1 p2距离就是 p1 到 p2的距离
                p1_to_p2 = Distance(p1, p2);
            } else {
                //p2 距离目标点 大于 p1到目标点距离，且p1追p2,这说明 p1在p2前，且同一个路径，p1要走一圈才能到p2很远
                double p1_goal_to_p2_pass = DistanceFromVectorID1ToVectorID2(sell_path_, robot_1_goal_point_id, robot_2_pass_point_id);
                if(p1_goal_to_p2_pass < 0.0){
                    return p1_goal_to_p2_pass;
                }
                double p1_to_goal = Distance(p1, sell_path_[robot_1_goal_point_id]);
                double p2_to_pass = Distance(p2, sell_path_[robot_2_pass_point_id]);
                p1_to_p2 = p1_goal_to_p2_pass + p1_to_goal + p2_to_pass;
            }
        } else {
                double p1_goal_to_p2_pass = DistanceFromVectorID1ToVectorID2(sell_path_, robot_1_goal_point_id, robot_2_pass_point_id);
                if(p1_goal_to_p2_pass < 0.0){
                    return p1_goal_to_p2_pass;
                }
                double p1_to_goal = Distance(p1, sell_path_[robot_1_goal_point_id]);
                double p2_to_pass = Distance(p2, sell_path_[robot_2_pass_point_id]);
                //ROS_INFO_STREAM("p1_goal_to_p2_pass " << p1_goal_to_p2_pass << " p1_goal " << p1_to_goal << " p2_pass " << p2_to_pass );
                
                p1_to_p2 = p1_goal_to_p2_pass + p1_to_goal + p2_to_pass;
        }
        dis = p1_to_p2;
        return true;
    }

int UpdateTwistByDistanceWithRemote(double& x_speed_change, const nav_msgs::Odometry& p1_ros) {
        if (is_get_remote && (robot_cnt > 1)) {
            //double ref_distance = 2.0 * (on_loop_distance / (float)robot_cnt);

            is_get_remote = false;

            double distance_p1_to_p2 = 9999.0;
            //动态计算间隔
            int on_path_robot = 1;
            for (int i = 0; i < remote_nav_point.size(); i++) {
                YoursNavPoint p2 = remote_nav_point[i];
                YoursNavPoint p1(p1_ros);
                double d_p1_p2 = 0.0;
                if(getDistanceFromRobot1ToRobot2(p1, p2, d_p1_p2)){
                    on_path_robot++;
                    if (d_p1_p2 < distance_p1_to_p2) {
                        distance_p1_to_p2 = d_p1_p2;
                    }
                }
            }

            if(distance_p1_to_p2 > 9990.0){
                distance_p1_to_p2 = -1.0;
            }


            log_->info(" 2 robot distance: " + std::to_string(distance_p1_to_p2));
            ROS_INFO_STREAM("2 robot distance : " << distance_p1_to_p2);
            x_speed_change = 0.0;
            if (distance_p1_to_p2 < 0) {
                // ROS_INFO_STREAM("distance_p1_to_p2 < 0");
                return -1;
            }

            if (on_loop_distance < 0) {
                // ROS_INFO_STREAM("on_loop_distance < 0");
                return -1;
            }

            if (distance_p1_to_p2 < stop_distance_) {
                ROS_INFO_STREAM("distance_p1_to_p2 " << stop_distance_);
                log_->info("distance_p1_to_p2 < " + std::to_string(stop_distance_));
                return 0;
            }


            double ref_distance = 2.0 * (on_loop_distance / (float)on_path_robot);


            if ((stop_distance_ < distance_p1_to_p2) && (distance_p1_to_p2 < (ref_distance / 3.0))) {
                x_speed_change = -0.07;  //减速
            }

            if (distance_p1_to_p2 > (2.0 * ref_distance / 4.0)) {
                x_speed_change = 0.07;  //加速
            }

            ROS_INFO_STREAM("update speed : " << x_speed_change);
            log_->info("update speed : " + std::to_string(x_speed_change));
            return 1;
        } else {
            return 3;
        }
    }



    bool InitWebGetPose();

 public:
    YoursGetGoalRobotDistance(ros::NodeHandle& n, const std::map<YoursPath, std::vector<YoursNavPoint>>& robot_path):robot_path_(robot_path),  on_loop_distance(0.0), is_get_remote(false), robot_cnt(2), stop_distance_(10.0) {
        ros::NodeHandle param_node("~");
        stop_distance_ = param_node.param<double>("stop_distance", 10.0);

        log_ = std::make_shared<yours_log::YoursLog>(std::string(getenv("HOME")) + "/log/yours_get_goal_robot_distance/");
        log_->set_log_level(yours_log::debug);
        log_->set_print_level(yours_log::error);
        
        if(robot_path_.count(Yours_Sell_Path_1) > 0){
            sell_path_ = robot_path_.at(Yours_Sell_Path_1);
        }
        if(sell_path_.empty()){
            log_->info("YoursGetGoalRobotDistance init error, no sell path 1");
        }else{
            log_->info("YoursGetGoalRobotDistance init ok");
        }

        on_loop_distance = DistanceFromVectorID1ToVectorID2(sell_path_, 0, (sell_path_.size() - 1));
        //ROS_INFO_STREAM("one_loop distance :" << on_loop_distance);
        log_->info(" one loop distance: " + std::to_string(on_loop_distance));
        //remote_pose_sub = n.subscribe("/remote_odom", 1, &YoursGetGoalRobotDistance::RemoteCallback, this);
        remote_pose_sub = n.subscribe("/remote_odom_array", 1, &YoursGetGoalRobotDistance::RemoteCallback, this);
        remote_robot_cnt_sub = n.subscribe("/remote_robot_cnt", 1, &YoursGetGoalRobotDistance::RobotCntCallback, this);
        
    };
    ~YoursGetGoalRobotDistance(){

    };

    int UpdateTwistByDistance(double& x_speed_change, const nav_msgs::Odometry& p1_ros) { return UpdateTwistByDistanceWithRemote(x_speed_change, p1_ros); }
    void setGoalPoint(YoursNavPoint goal);
};

#endif
