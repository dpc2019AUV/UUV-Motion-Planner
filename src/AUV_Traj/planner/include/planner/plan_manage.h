#ifndef PLAN_MANAGE_H
#define PLAN_MANAGE_H

#include <iostream>
#include <random>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <traj_msg/TrajCmd.h>

#include "map/map_manage.h"
#include "path_planning/path_planning.h"
#include "traj_opt/traj_opt.h"

using namespace Eigen;
using namespace std;

enum STATE {WAITE_WP, GEN_TRAJ, REPLAN_TRAJ, EXE_TRAJ};

class PlanManage
{
    public:
        PlanManage(ros::NodeHandle& nh);
        ~PlanManage();
        void init();
        void WPCallBack(const nav_msgs::Path& msg);
        //void goalRcvCallBack(const geometry_msgs::PoseStamped& msg);
        void odomRcvCallBack(const traj_msg::TrajCmd& msg);
        void changeState(STATE new_state);
        void fsmCallBack(const ros::TimerEvent& e);
        void collisionCallBack(const ros::TimerEvent& e);
        void mapRcvCallBack(const sensor_msgs::PointCloud2& globalmap);
        Vector3d set_vel(const Vector3d angle, const double vel);
        void getWP(vector<Vector3d>& path, vector<Vector3d>& wp, bool& fixed_goal);
        bool traj_planning(bool replan_flag);
        void rand_disturb(vector<Vector3d>& path, bool flag);
        void time_alloc(const vector<Vector3d>& path, vector<double>& dur, bool replan_flag);
        void display_pose_arrow(const Vector3d& odom_p_, const Vector3d& odom_angle_, const Vector3d& goal_p_, const Vector3d& goal_angle_);
        void display_wp(const vector<Vector3d> path);
        void changeState();
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        ros::NodeHandle nh_;
        ros::Timer fsm_, collision_check_;
        ros::Subscriber goal_sub_, map_sub_, odom_sub_;
        ros::Subscriber waypoints_sub_;
        ros::Publisher pose_arrow_vis_pub_, wptemp_vis_pub_;
        ros::Publisher vel_pub_, acc_pub_;
        Eigen::Vector3d odom_p_, odom_v_, odom_a_, odom_angle_; 
        Eigen::Vector3d goal_p_, goal_v_, goal_a_, goal_angle_;
        bool has_start_, has_goal_, has_map_, has_odom_;
        double dist2start_triger_, dist2goal_triger_, v_mean_;
        int fail_count_;
        bool disturbance_triger_;

    public:
        MapManage::Ptr map_manage_ptr_;
        PathPlanning* path_planning_ptr_;
        TrajOpt* traj_opt_ptr_;
        STATE cur_state_;
        Vector3d replan_pt_;

};

#endif   // PLAN_MANAGE_H