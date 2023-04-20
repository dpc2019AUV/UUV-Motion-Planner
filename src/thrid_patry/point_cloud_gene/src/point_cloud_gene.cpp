#include <ros/ros.h>
#include <Eigen/Eigen> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
//#include <stdlib.h>
//#include <time.h>
#include <vector>
#include <iostream>
#include <random>

using namespace Eigen;
using namespace std;

ros::Publisher global_map_pub;
double cloud_resolution, dist_safe;
int obs_number;
double map_xsize, map_ysize, map_zsize;
pcl::PointCloud<pcl::PointXYZ> global_map_pcl_cloud;
//sensor_msgs::PointCloud2 global_map_cloud;
std::vector<Vector2d> obs_pt; 
Vector3d origin, map_lower, map_upper; 

void geneWall(double ori_x, double ori_y, double length, double width, double height)
{
    pcl::PointXYZ s_point;
    for( double t_z = 0.5; t_z <= height ; t_z += cloud_resolution )
    {
        for( double t_y = -width; t_y <= width ; t_y += cloud_resolution )
        {
            for( double t_x = -length; t_x <= length; t_x += cloud_resolution)
            {
                s_point.x = ori_x + t_x;
                s_point.y = ori_y + t_y;
                s_point.z = t_z;
                //cout << "s_point.x = " << s_point.x << " " << "s_point.y = " << s_point.y << endl; 

                if ( (s_point.x >= map_lower(0)) && (s_point.x <= map_upper(0)) && (s_point.y >= map_lower(1)) && (s_point.y <= map_upper(1)) )
                    global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void PC2Gene()
{
    // srand((unsigned)time(NULL));   //******
    //generate random point cloud
    obs_pt.clear();
    while (obs_pt.size() < obs_number)
    {
        double pt_x = (1 + rand() % (int)(map_xsize*10)) / 10.0 + origin(0);
        double pt_y = (1 + rand() % (int)(map_ysize*10)) / 10.0 + origin(1);
        // cout << "pt_x = " << pt_x << endl;
        // cout << "pt_y = " << pt_y << endl;
        Vector2d pt(pt_x, pt_y);
        if (obs_pt.size() == 0)
        {
            obs_pt.push_back(pt);
        }
        else
        {
            double min_dist = sqrt( pow(map_xsize, 2.0) + pow(map_ysize, 2.0) + pow(map_zsize, 2.0) );
            for (int i=0; i<obs_pt.size(); i++)
            {
                double xtemp = obs_pt[i](0);
                double ytemp = obs_pt[i](1); 
                double L2_square = sqrt(pow(pt_x - xtemp, 2.0) + pow(pt_y - ytemp, 2.0));
                if (L2_square < min_dist)
                {
                    min_dist = L2_square;
                }
            }
            if (min_dist > dist_safe)
            {
                obs_pt.push_back(pt);
            }
        }
    }
    for (int i=0; i<obs_pt.size(); i++)
    {
        double x = obs_pt[i](0);
        double y = obs_pt[i](1);
        double width = 2.0 + rand()%2;
        double length = 2.0 + rand()%2;
        int height_base = (int)(map_zsize/2.0);
        double heigth = map_zsize/4.0 + rand() % height_base;
        //double heigth = 20.0;
        geneWall(x, y, width, length, heigth);
    }
    global_map_pcl_cloud.width = global_map_pcl_cloud.points.size();
    global_map_pcl_cloud.height = 1;
    global_map_pcl_cloud.is_dense = true;
}

// void pub_pcl()
// {
//     globalpoint_ros.header.frame_id = "world";
//     gobal_point_pub.publish(globalpoint_ros);
//     center_pub.publish(center);
//     range_pub.publish(sphere);
//     selectpoint_ros.header.frame_id = "world";
//     select_point_pub.publish(selectpoint_ros);
//     cout << "has_pub" << endl;
// }

void pubGlobalMap()
{
    sensor_msgs::PointCloud2 global_map_cloud;
    pcl::toROSMsg(global_map_pcl_cloud, global_map_cloud);
    global_map_cloud.header.frame_id = "world";
    global_map_pub.publish(global_map_cloud);
    ROS_INFO("global map published!");
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "globalmap_generator"); 
    ros::NodeHandle nh("~");  //nh("~")
    nh.param("pc/cloud_resolution", cloud_resolution, 1.0);
    nh.param("pc/obs_number", obs_number, -1);
    nh.param("pc/distacne_safe", dist_safe, -1.0);
    nh.param("pc/map_xsize", map_xsize, -1.0);
    nh.param("pc/map_ysize", map_ysize, -1.0);
    nh.param("pc/map_zsize", map_zsize, -1.0);

    map_lower << -map_xsize/2, -map_ysize/2, 0;
    map_upper << map_xsize/2, map_ysize/2, map_zsize; 
    origin = map_lower;
    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 10);

    PC2Gene(); 

    ros::Rate rate(0.8);   
    int t = 5;
    while(t--)
    {
        //pub_pcl();
        pubGlobalMap();
        rate.sleep();
    }
    return 0;
}




