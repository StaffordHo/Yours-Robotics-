#include <ros/ros.h>
#include "../include/yours_navigation/plan.h"
#include "../include/yours_navigation/slamDaemon.h"
#include "../include/yours_navigation/yours_vision_detect_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yours_navigation");
    ros::NodeHandle n("~");
    std::string path;
    std::string PATH;
    std::string mLocateMode;
    bool isZhiheng = false;
    bool isTebTest = false;
    bool isWithMoveBase = false;
    bool isDockTest = false;
    bool isRemapMode = false;
    n.param("locate_mode", mLocateMode, std::string("null"));
    n.param("zhiheng", isZhiheng, false);
    n.param("teb_test",isTebTest, false);
    n.param("teb_with_move_base",isWithMoveBase, false);
    n.param("dock_test", isDockTest, false);
    n.param("remap_mode", isRemapMode, false);
    ros::Rate rate(10);
    //std::shared_ptr<Plan> plan = std::make_shared<Plan>(n);
    sleep(10);
    Plan plan(n, isRemapMode);
    yours_vision_detect_node yVision(n);
     
    while (ros::ok()) {

        if (isDockTest) {
            plan.RunChargeTest();
		}
		else if(isRemapMode){
            plan.RunRemap();
        } else {
            if (!isTebTest) {
                if (isZhiheng) {
                    plan.RunZH();
                } else {
                    if (mLocateMode.compare("amcl") == 0) {
                        plan.TestAmclRun();
                    } else {
                        plan.Run();
                    }
                }
            } else {
                if (isWithMoveBase) {
                    plan.testTebPlan();
                } else {
                    plan.TestLocalPlan();
                }
            }
        }

        // plan.testDangerZone();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
