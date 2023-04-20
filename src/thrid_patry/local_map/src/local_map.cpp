#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h> 
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "traj_msg/TrajCmd.h"

using namespace Eigen;
using namespace std;

ros::Publisher localMap_pub;
pcl::PointCloud<pcl::PointXYZ> cloud_input;
pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
sensor_msgs::PointCloud2 local_map_pcd;

Vector3d odom_p, odom_angle;
geometry_msgs::Quaternion odom_quat;

bool has_global_map = false;
bool has_odom = false;
bool pub_local_map = false;
bool update_odom = false;
double sensor_range, sensor_rate;

void globalPointCallBack(const sensor_msgs::PointCloud2& global_map)
{
    if(has_global_map) return;
    ROS_WARN("Global Pointcloud received..");

    pcl::fromROSMsg(global_map, cloud_input);
    cloud_input.width = cloud_input.points.size();
    cloud_input.height = 1;
    cloud_input.is_dense = true;
    kdtreeLocalMap.setInputCloud(cloud_input.makeShared());
    has_global_map = true;
}

void start_ptCallBack(const nav_msgs::Path& wp)
{
    vector<Vector3d> wp_list;
    vector<Vector3d> pose_list;
    wp_list.clear();

    odom_p << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z;
    odom_quat = wp.poses[0].pose.orientation;
    //cout << "local_map, odom_p = " << odom_p.transpose() << endl;

    has_odom = true;
}

void localMapCallBack(const ros::TimerEvent& event)
{
    if(!has_global_map || !has_odom) return;

    Eigen::Quaterniond q;
    q.x() = odom_quat.x;
    q.y() = odom_quat.y;
    q.z() = odom_quat.z;
    q.w() = odom_quat.w;

    Matrix3d rot;
    rot = q;
    Vector3d yaw_vec = rot.col(0);

    pcl::PointCloud<pcl::PointXYZ> local_map;
    local_map.points.clear();
    pcl::PointXYZ searchPoint(odom_p(0), odom_p(1), odom_p(2));
    vector<int> pointIdx;
    vector<float> pointRadius;
    pointIdx.clear();
    pointRadius.clear();

    pcl::PointXYZ pt;
    int neighbourcount;
    neighbourcount = kdtreeLocalMap.radiusSearch(searchPoint, sensor_range, pointIdx, pointRadius);
    if (neighbourcount > 0) 
    {
        for (size_t i = 0; i < pointIdx.size(); ++i) 
        {
            pt = cloud_input.points[pointIdx[i]];

            Vector3d pt_vec(pt.x - odom_p(0), pt.y - odom_p(1), pt.z - odom_p(2));
            // if(abs(pt.z-odom_p(2)) > 3.0) continue;
            if(pt.z-odom_p(2) > 3.0) continue;
            if(pt_vec.dot(yaw_vec) < 0) continue;
            local_map.points.push_back(pt);
        }
    } 
    else 
    {
        return;
    }
    local_map.width = local_map.points.size();
    local_map.height = 1;
    local_map.is_dense = true;

    sensor_msgs::PointCloud2 local_map_pcd;
    pcl::toROSMsg(local_map, local_map_pcd);
    local_map_pcd.header.frame_id = "world";
    localMap_pub.publish(local_map_pcd);
    pub_local_map = true;
    //ROS_WARN("local_map has published!");
}

void odomCallBack(const traj_msg::TrajCmd& cmd)
{
    //cout << "local_map, into odomCallBack() function" << endl;
    odom_p(0) = cmd.pt.x; 
    odom_p(1) = cmd.pt.y; 
    odom_p(2) = cmd.pt.z; 

    double vx = cmd.vel.x;
    double vy = cmd.vel.y;
    double vz = cmd.vel.z;
    double vh = sqrt(pow(vx,2) + pow(vy,2));

    double yaw = atan2(vy, vx);
    double pitch = atan2(vz, vh);
    double roll = 0.0;

    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    //update_odom = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map");
    ros::NodeHandle nh("~");
    nh.param("loacl_map/sensor_range", sensor_range, -1.0);
    nh.param("local_map/sensor_rate", sensor_rate, -1.0);

    ros::Subscriber gloablMap_sub = nh.subscribe("/point_cloud", 1, &globalPointCallBack);
    ros::Subscriber start_pt_sub = nh.subscribe("/waypoints", 1, &start_ptCallBack);
    ros::Subscriber odom_sub = nh.subscribe("/server/cmd", 1, &odomCallBack);

    localMap_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 10);
    ros::Timer local_map_timer = nh.createTimer(ros::Duration(0.05), &localMapCallBack);

    ros::Rate r(100);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}