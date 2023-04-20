#include "planner/plan_manage.h"

PlanManage::~PlanManage()
{
    delete traj_opt_ptr_;
    delete path_planning_ptr_;
    //delete map_manage_ptr_;
}

PlanManage::PlanManage(ros::NodeHandle& nh)
{
    cout << "configuration function" << endl;
    nh_ = nh;
    has_goal_ = false;
    has_start_ = false;
    has_map_ = false;

    fail_count_ = 0;
    disturbance_triger_ = false;
    cur_state_ = WAITE_WP;
}

void PlanManage::init()
{

    nh_.param("plan_man/v_mean", v_mean_, -0.1);
    nh_.param("plan_man/dist2start", dist2start_triger_, 4.0);
    nh_.param("plan_man/dist2goal", dist2goal_triger_, 10.0);


    map_manage_ptr_.reset(new MapManage);
    map_manage_ptr_ -> init(nh_);
    path_planning_ptr_ = new PathPlanning;
    path_planning_ptr_ -> setmap(map_manage_ptr_);
    path_planning_ptr_ -> init(nh_);
    traj_opt_ptr_ = new TrajOpt;
    traj_opt_ptr_ -> setmap(map_manage_ptr_);
    traj_opt_ptr_ -> init(nh_); 

    fsm_ = nh_.createTimer(ros::Duration(0.01), &PlanManage::fsmCallBack, this);
    collision_check_ = nh_.createTimer(ros::Duration(1), &PlanManage::collisionCallBack, this);
    map_sub_ = nh_.subscribe("/local_map", 10, &PlanManage::mapRcvCallBack, this);
    waypoints_sub_ = nh_.subscribe("/waypoints", 10, &PlanManage::WPCallBack, this);
    odom_sub_ = nh_.subscribe("/server/cmd", 1, &PlanManage::odomRcvCallBack, this);


    pose_arrow_vis_pub_  = nh_.advertise<visualization_msgs::MarkerArray>("/PlanMan/pose_arrow_vis", 1);
    wptemp_vis_pub_  = nh_.advertise<visualization_msgs::Marker>("/PlanMan/wptemp_vis", 1);
}

void PlanManage::changeState(STATE new_state)
{
    string state_str[4] = {"WAITE_WP", "GEN_TRAJ", "REPLAN_TRAJ", "EXE_TRAJ"};
    int pre_s = int(cur_state_);
    cur_state_ = new_state;
    cout << "[ FINITE STATE MACHINE ]: from " + state_str[pre_s] + " to " + state_str[int(cur_state_)] << endl;
}






void PlanManage::mapRcvCallBack(const sensor_msgs::PointCloud2& sensor_point)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(sensor_point, cloud);

    map_manage_ptr_ -> get_grid_map(cloud);
    map_manage_ptr_ -> get_sdf();
    if (map_manage_ptr_ -> has_gridmap())
        has_map_ = true;
}

Vector3d PlanManage::set_vel(const Vector3d angle, const double vel)
{
    Vector3d vel_vec;
    double yaw;
    yaw = angle(2);
    vel_vec(0) = vel * cos(yaw);
    vel_vec(1) = vel * sin(yaw);
    vel_vec(2) = 0;
    return vel_vec;
}


void PlanManage::getWP(vector<Vector3d>& path, vector<Vector3d>& wp, bool& reach_goal)
{
    path.insert(path.begin(), odom_p_);
    wp.clear();
    wp.push_back(path[0]);
    reach_goal = false;
    for(auto i=1; i<path.size(); i++)
    {
        if((path[i]-path[0]).norm() < 15.0)
            wp.push_back(path[i]);
        else
            break;
    }
    if((wp.back() - goal_p_).norm() < 0.1 )
    {
        reach_goal = true;
    }
    path_planning_ptr_ -> simplify_path(wp);
    path_planning_ptr_ -> interpolate_wp(wp);
}

void PlanManage::rand_disturb(vector<Vector3d>& path, bool flag)
{
    int n;
    vector<Vector3d> wp = path;
    n = path.size()-1;
    random_device rd;
    mt19937 r_eng(rd());
    uniform_real_distribution<double> dis(-2, 2);
    for(int i=1; i<n; i++)
    {
        bool occupany = true;
        do
        {
            path[i] = wp[i];
            Vector3d disturbance;
            disturbance << dis(r_eng), dis(r_eng), dis(r_eng)/4;
            path[i] += disturbance;

            Vector3i pt_id = map_manage_ptr_ -> coord2index(path[i]);
            int obs_id = map_manage_ptr_ -> voxel_id(pt_id);
            occupany =  map_manage_ptr_ -> obs_buffer_[obs_id];
        }
        while(occupany);
    }
    disturbance_triger_ = false;
}

void PlanManage::time_alloc(const vector<Vector3d>& path, vector<double>& dur, bool flag)
{
    dur.clear();
    double v_mean;

    dur.reserve(path.size()-1);
    for(int i=0; i<path.size()-1; i++)
    { 
        double dist = (path[i+1] - path[i]).norm();
        double t = dist / v_mean_;
        dur.push_back(t);
    }
    if(!flag)
    {
        dur.front() *= 1.3;
    }
    dur.back() *= 1.5;
}

void PlanManage::display_pose_arrow(const Vector3d& odom_p_, const Vector3d& odom_angle_, const Vector3d& goal_p_, const Vector3d& goal_angle_)
{
    vector<Vector3d> pt;
    vector<Vector3d> angle;
    pt.clear();
    angle.clear();
    pt.push_back(odom_p_);
    pt.push_back(goal_p_);
    angle.push_back(odom_angle_);
    angle.push_back(goal_angle_);

    visualization_msgs::MarkerArray wps_vis;
    wps_vis.markers.clear();
    visualization_msgs::Marker wp_vis;
    double color[2][4] = {{1.0, 0.0, 1.0, 0.3}, {0.0, 1.0, 0.0, 0.3}};
    for (int i = 0; i < (int)pt.size(); i++)
    {
        geometry_msgs::Quaternion q;
        double roll = angle[i](0);
        double pitch = angle[i](1);
        double yaw = angle[i](2);
        q=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

        wp_vis.type = visualization_msgs::Marker::ARROW;
        wp_vis.header.stamp = ros::Time::now();
        wp_vis.header.frame_id = "/world";
        wp_vis.ns = "pose_arrow";
        wp_vis.id = i;
        wp_vis.scale.x = 2;
        wp_vis.scale.y = 0.5;
        wp_vis.scale.z = 0.2;

        wp_vis.pose.orientation = q;
        wp_vis.pose.position.x = pt[i](0);
        wp_vis.pose.position.y = pt[i](1);
        wp_vis.pose.position.z = pt[i](2);

        wp_vis.color.r = color[i][0];
        wp_vis.color.g = color[i][1];
        wp_vis.color.b = color[i][2];
        wp_vis.color.a = color[i][3];
        wps_vis.markers.push_back(wp_vis);
    }
    pose_arrow_vis_pub_.publish(wps_vis);
}

void PlanManage::display_wp(const vector<Vector3d> path)
{
    visualization_msgs::Marker wptemp_vis;
    wptemp_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    wptemp_vis.action = visualization_msgs::Marker::DELETEALL;
    wptemp_vis_pub_.publish(wptemp_vis);


    wptemp_vis.action = visualization_msgs::Marker::ADD;
    wptemp_vis.header.stamp = ros::Time::now();
    wptemp_vis.header.frame_id = "world";
    wptemp_vis.lifetime = ros::Duration();
    wptemp_vis.id = 0;

    wptemp_vis.scale.x = 0.8;
    wptemp_vis.scale.y = 0.8;
    wptemp_vis.scale.z = 0.8;

    wptemp_vis.pose.orientation.x = 0.0;
    wptemp_vis.pose.orientation.y = 0.0;
    wptemp_vis.pose.orientation.z = 0.0;
    wptemp_vis.pose.orientation.w = 1.0;

    wptemp_vis.color.r = 0.0;
    wptemp_vis.color.g = 1.0;
    wptemp_vis.color.b = 1.0;
    wptemp_vis.color.a = 0.5;

    geometry_msgs::Point pt;
    for(int i=0; i<path.size(); i++)
    {
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        wptemp_vis.points.push_back(pt);
    }
    wptemp_vis_pub_.publish(wptemp_vis);
}

void PlanManage::odomRcvCallBack(const traj_msg::TrajCmd& msg)   
{
    odom_p_(0) = msg.pt.x;
    odom_p_(1) = msg.pt.y;
    odom_p_(2) = msg.pt.z;

    odom_v_(0) = msg.vel.x;
    odom_v_(1) = msg.vel.y;
    odom_v_(2) = msg.vel.z;

    odom_a_(0) = msg.acc.x;
    odom_a_(1) = msg.acc.y;
    odom_a_(2) = msg.acc.z;

    double yaw = atan2(odom_v_(1), odom_v_(0));
    Vector2d vh(odom_v_(0), odom_v_(1));
    double v_h = vh.norm();
    double pitch = atan2(odom_v_(2), v_h);
    double roll = 0.0;

    odom_angle_ << roll, pitch, yaw;
    //cout << "current_p = " << odom_p_.transpose() << endl;
}

// void PlanManage::goalRcvCallBack(const geometry_msgs::PoseStamped& msg)
// {
//     if (msg.pose.position.z < -0.5)
//     {
//         return;
//     }
//     goal_p_(0) = msg.pose.position.x;
//     goal_p_(1) = msg.pose.position.y;
//     goal_p_(2) = msg.pose.position.z;

//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(msg.pose.orientation, quat);
//     tf::Matrix3x3(quat).getRPY(goal_angel_(0), goal_angel_(1), goal_angel_(2));

//     has_goal_ = true;
//     cout << "goal = " << goal_p_.transpose() << endl;

//     if (has_goal_ && has_odom_ && has_map_)
//     {
//         has_odom_ = has_goal_ = false;
//         cout << "calculateing trajectory ......." << endl;
//     }
// }





