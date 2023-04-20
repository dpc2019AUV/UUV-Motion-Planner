#include <ros/ros.h>
#include "planner/plan_manage.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh("~");

    PlanManage planmanage(nh);
    planmanage.init();

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
