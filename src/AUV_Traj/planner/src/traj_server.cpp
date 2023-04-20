#include <iostream>
#include <ros/ros.h>
#include <traj_opt/TrajInfo.h>
#include <traj_msg/TrajCmd.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64.h>
#include <fstream>

#include "traj_opt/traj_opt.h"
using namespace std;

ros::Publisher cmd_pub, VelCmd_vis_pub, velNorm_pub, accNorm_pub, pt_vis_pub, odom_vis_pub;
visualization_msgs::Marker vel_vis, odom_vis;
traj_opt::TrajInfo traj;
MatrixXd coe;

bool has_traj = false;
bool clear_traj = false;
double dur, t_total;
vector<Vector3d> pt_walked;

void pub(Vector3d& des_pt, Vector3d& des_vel, Vector3d des_acc, ros::Time& t_cur, int& id)
{
    traj_msg::TrajCmd cmd;
    cmd.pt.x = des_pt(0);
    cmd.pt.y = des_pt(1);
    cmd.pt.z = des_pt(2);
    cmd.vel.x = des_vel(0);
    cmd.vel.y = des_vel(1);
    cmd.vel.z = des_vel(2);
    cmd.acc.x = des_acc(0);
    cmd.acc.y = des_acc(1);
    cmd.acc.z = des_acc(2);
    cmd.header.stamp = t_cur;
    cmd.header.frame_id = "/world";
    cmd.traj_Id = 0;
    cmd_pub.publish(cmd);
    pt_walked.push_back(des_pt);

    double vel, acc;
    vel = des_vel.norm();
    acc = des_acc.norm();
    std_msgs::Float64 v, a;
    v.data = vel;
    a.data = acc;
    velNorm_pub.publish(v);
    accNorm_pub.publish(a);

    //vel_vis_pub
    vel_vis.action = visualization_msgs::Marker::ADD;    
    vel_vis.type = visualization_msgs::Marker::ARROW;
    vel_vis.header.stamp = t_cur;
    vel_vis.header.frame_id = "world";
    vel_vis.lifetime = ros::Duration();
    vel_vis.ns = "vel_vis";
    vel_vis.id = 0;

    vel_vis.scale.x = 0.3;
    vel_vis.scale.y = 0.6;
    vel_vis.scale.z = 0.9;
    vel_vis.pose.orientation.w = 1.0;

    vel_vis.points.clear();
    geometry_msgs::Point pt;
    pt.x = des_pt(0);
    pt.y = des_pt(1);
    pt.z = des_pt(2);
    vel_vis.points.push_back(pt);
    pt.x = des_pt(0) + 2*des_vel(0);
    pt.y = des_pt(1) + 2*des_vel(1);
    pt.z = des_pt(2) + 2*des_vel(2);
    vel_vis.points.push_back(pt);

    vel_vis.color.r = 1.0;
    vel_vis.color.g = 1.0;
    vel_vis.color.b = 0.0;
    vel_vis.color.a = 1.0;
    VelCmd_vis_pub.publish(vel_vis);

    odom_vis.type = visualization_msgs::Marker::SPHERE;
    odom_vis.header.stamp = t_cur;
    odom_vis.header.frame_id = "world";
    odom_vis.ns = "odom_vis";
    odom_vis.scale.x = 0.8;
    odom_vis.scale.y = 0.8;
    odom_vis.scale.z = 0.8;
    odom_vis.pose.orientation.w = 1.0;
    odom_vis.pose.position.x = des_pt(0);
    odom_vis.pose.position.y = des_pt(1);
    odom_vis.pose.position.z = des_pt(2);
    odom_vis.color.r = 1.0;
    odom_vis.color.g = 0.0;
    odom_vis.color.b = 0.0;
    odom_vis.color.a = 1.0;
    odom_vis_pub.publish(odom_vis);

}

void TrajCallbcak(const traj_opt::TrajInfo& msg)
{
    t_total = 0.0;
    for(int i=0; i < msg.time.size(); i++)
    {
        t_total += msg.time[i];
    }
    cout << "traj_server, total_time = " << t_total << endl;
    int m = msg.coeff.size();
    coe = MatrixXd::Zero(m,3);
    for(int i=0; i < msg.coeff.size(); i++)
    {
        coe(i,0) = msg.coeff[i].x;
        coe(i,1) = msg.coeff[i].y;
        coe(i,2) = msg.coeff[i].z;
    }
    traj = msg;
    has_traj = true;
    if(clear_traj)
        pt_walked.clear();
    clear_traj = false;

    ROS_WARN("[Traj server]: ready ");
    // VelCmd_vis_pub.publish(vel_van);
}

void cmdCallback(const ros::TimerEvent &e)
{
    if(!has_traj)
        return;
    int id = 0;
    int j = 0;
    double t = 0;
    double t_rel = 0.0;
    ros::Time t_cur = ros::Time::now();
    dur = (t_cur - traj.start_time).toSec();
    if(/*has_traj && */dur <= t_total && dur >= 0)
    {
        for(int i=0; i<traj.time.size(); i++)
        {
            t_rel += traj.time[i];
            if(t_rel - dur > 0)
            {
                j = i;
                t = dur - t_rel + traj.time[i];
                break;
            }
        }
        Vector3d pt(0, 0, 0), vel(0, 0, 0), acc(0, 0, 0);
        Matrix<double, 6, 3> coe_cur = coe.block(6*j, 0, 6, 3);
        for(int m=0; m<3; m++)
        {
            for(int n=0; n<6; n++)
            {
                pt(m) += coe_cur.col(m)(n) * pow(t, n);
                if(n >= 1)
                    vel(m) += n * coe_cur.col(m)(n) * pow(t, n-1);
                if(n >= 2)
                    acc(m) += n * (n-1) * coe_cur.col(m)(n) * pow(t, n-2);
            }
        }
        pub(pt, vel, acc, t_cur, id);
    }
    else
    {
        //end execute traj, set variable
        id = 0;
        has_traj = false;
        clear_traj = true;
    }
}

void traj_visCallback(const ros::TimerEvent &e)
{
    visualization_msgs::Marker pt_vis;
    pt_vis.header.frame_id = "world";
    pt_vis.header.stamp = ros::Time::now();
    pt_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    pt_vis.action = visualization_msgs::Marker::DELETE;
    pt_vis.id = 0;

    pt_vis_pub.publish(pt_vis);

    pt_vis.action = visualization_msgs::Marker::ADD;
    pt_vis.pose.orientation.x = 0.0;
    pt_vis.pose.orientation.y = 0.0;
    pt_vis.pose.orientation.z = 0.0;
    pt_vis.pose.orientation.w = 1.0;

    pt_vis.color.r = 0.0;
    pt_vis.color.g = 1.0;
    pt_vis.color.b = 0.0;
    pt_vis.color.a = 1.0;

    pt_vis.scale.x = 0.2;
    pt_vis.scale.y = 0.2;
    pt_vis.scale.z = 0.2;

    geometry_msgs::Point pt;
    for (int i = 0; i < pt_walked.size(); i++) 
    {
        pt.x = pt_walked[i](0);
        pt.y = pt_walked[i](1);
        pt.z = pt_walked[i](2);
        pt_vis.points.push_back(pt);
    }
    pt_vis_pub.publish(pt_vis);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle node("~");

    cmd_pub = node.advertise<traj_msg::TrajCmd>("/server/cmd", 10);
    VelCmd_vis_pub = node.advertise<visualization_msgs::Marker>("/server/des_vel", 1);
    velNorm_pub = node.advertise<std_msgs::Float64>("/server/velNorm", 10);
    accNorm_pub = node.advertise<std_msgs::Float64>("/server/accNorm", 10);





    
    pt_vis_pub = node.advertise<visualization_msgs::Marker>("/server/pt_vis", 1);
    odom_vis_pub = node.advertise<visualization_msgs::Marker>("/server/odom_vis", 1);

    ros::Subscriber traj_sub = node.subscribe("/traj_opt/traj_info", 10, TrajCallbcak);
    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
    ros::Timer traj_vis_timer = node.createTimer(ros::Duration(0.2), traj_visCallback); 

    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}



