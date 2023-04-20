#include "traj_opt/traj_opt.h"

TrajOpt::TrajOpt(){};
TrajOpt::~TrajOpt()
{
    //delete map_ptr_;
};

void TrajOpt::init(ros::NodeHandle& nh)
{
    nh.param("traj/order", order_, 5);
    nh.param("traj/v_mean", v_mean_, 0.95);
    nh.param("traj/v_max", v_max_, 1.0);
    nh.param("traj/a_max", a_max_, 0.5);
    nh.param("traj/wh_max", wh_max_, 0.4);
    nh.param("traj/wv_max", wv_max_, 0.25);
    nh.param("traj/cur_max", cur_max_, 0.4);
    nh.param("traj/p_max", p_max_, M_PI/6);
    nh.param("traj/safe_dist", safe_dist_, 4.0);

    traj_vis_flag_ = false;
    coeff_num_ = order_ + 1;

    //odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    traj_pub_ = nh.advertise<traj_opt::TrajInfo>("/traj_opt/traj_info", 1);
    traj_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/traj_opt/traj_vis", 1);
    vel_pub_ = nh.advertise<std_msgs::Float64>("/traj_opt/vel", 1);
    acc_pub_ = nh.advertise<std_msgs::Float64>("/traj_opt/acc", 1);
}

void TrajOpt::setmap(const MapManage::Ptr& env)
{
    map_ptr_ = env;
}

void TrajOpt::reset()
{
    dur_.clear();
    wp_.clear();
}

double TrajOpt::get_time()
{
    return t_total_;
}



void TrajOpt::get_Q(const int seg_num)
{
    int n = seg_num;
    int m = coeff_num_;
    for(int k=0; k<n; k++)
    {
        for(int i=3; i<m; i++)
        {
            for(int j=3; j<m; j++)
            {
                Q_(k*m + i, k*m + j) = i*(i-1)*(i-2)*j*(j-1)*(j-2)/(i+j-5)*pow(dur_[k], (i+j-5));
            }
        }
    }
}

void TrajOpt::get_M(const int seg_num)
{
    int n = seg_num;
    int m = coeff_num_;
    for(int k=0; k<n; k++)
    {
        MatrixXd M_sub = MatrixXd::Zero(m, m);
        for(int i=0; i<3; i++)
        {
            M_sub(i,i) = Factorial(i);
            for(int j=i; j<m; j++)
            {
                M_sub(3+i, j) = Factorial(j) / Factorial(j-i) * pow(dur_[k], j-i);
            }
        }
        M_.block(k*m, k*m, m, m) = M_sub;
    }
}

// fixed first and last waypoints
void TrajOpt::get_C(const int seg_num)
{
    int n = seg_num;
    int m = coeff_num_;  //6
    MatrixXd Ct = MatrixXd::Zero(n*m, 3*(n+1));
    if(opt_state_ == GOAL)
    {
        for(int i=0; i<3; i++)
        {
            Ct(i,i) = 1;
            Ct(6*n-3+i,3+i) = 1;
        }
        for(int i=0; i<n-1; i++)
        {
            for(int j=0; j<3; j++)
            {
                Ct(3+6*i+j,6+3*i+j) = 1;
                Ct(6+6*i+j,6+3*i+j) = 1;            
            }

        }
    }
    else if(opt_state_ == HORIZON)
    {
        for(int i=0; i<3; i++)
        {
            Ct(i,i) = 1;
        }
        Ct(6*n-3,3) = 1;
        for(int i=0; i<n-1; i++)
        {
            for(int j=0; j<3; j++)
            {
                Ct(3+6*i+j, 4+3*i+j) = 1;
                Ct(6+6*i+j, 4+3*i+j) = 1;
            }
        }
        for(int i=0; i<2; i++)
        {
            Ct(6*n-2+i, (n+1)*3-2+i) = 1;
        }
    }
    else
    {}
    C_ = Ct.transpose();
}

void TrajOpt::getWP(const MatrixXd& dp, const MatrixXd& df)
{
    MatrixXd coe = get_coeff(dp);
    wp_.clear();
    Vector3d pt = get_pos(coe, 0, 0);
    wp_.push_back(pt);
    for(int i=0; i<dur_.size(); i++)
    {
        Vector3d pt = get_pos(coe, i, dur_[i]);
        wp_.push_back(pt);
    }
}




vector<double> TrajOpt::get_dura()
{
    return this -> dur_;
}



double TrajOpt::costFunc(void* ptr, const VectorXd& x, VectorXd& grad)
{
    //TrajOpt &trajopt = *(TrajOpt *)ptr;
    TrajOpt *trajopt = reinterpret_cast<TrajOpt *>(ptr);
    double cost;
    trajopt -> getCostGrad(x, cost, grad);
    return cost;
}



MatrixXd TrajOpt::get_coeff(const MatrixXd& dp)
{
    int df_num = df_.rows(); 
    int dp_num = dp.rows();
    MatrixXd df_dp = MatrixXd::Zero(df_num + dp_num, 3);
    df_dp.block(0, 0, df_num, 3) = df_;
    df_dp.block(df_num, 0, dp_num, 3) = dp;

    MatrixXd coe = MatrixXd::Zero(6*dur_.size(), 3);
    for(int i=0; i<3; i++)
    {
        coe.col(i).noalias() = L_ * df_dp.col(i);
    }
    return coe;
}

Vector3d TrajOpt::get_pos(double& dur)
{
    double t = 0;
    int t_rel = 0;
    int j = 0;
    if(dur <= t_total_)
    {
        for(int i=0; i<dur_.size(); i++)
        {
            t_rel += dur_[i];
            if(t_rel - dur > 0)
            {
                j = i;
                t = dur - t_rel + dur_[i];
                break;
            }
        }
    }
    Vector3d cur_pt = get_pos(coeff_, j, t);
    return cur_pt;
}

Vector3d TrajOpt::get_pos(const MatrixXd& coe, const int m, const double t)
{
    Vector3d pos(0,0,0);
    Matrix<double, 6, 3> coe_temp = coe.block(6*m, 0, 6, 3);
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<6; j++)
        {
            pos(i) += coe_temp.col(i)(j) * pow(t, j);
        }
    }
    return pos;
}

Vector3d TrajOpt::get_vel(double& dur)
{
    double t = 0;
    int t_rel = 0;
    int j = 0;
    if(dur <= t_total_)
    {
        for(int i=0; i<dur_.size(); i++)
        {
            t_rel += dur_[i];
            if(t_rel - dur > 0)
            {
                j = i;
                t = dur - t_rel + dur_[i];
                break;
            }
        }
    }
    Vector3d cur_vel = get_vel(coeff_, j, t);
    return cur_vel;
}

Vector3d TrajOpt::get_vel(const MatrixXd& coe, const int m, const double t)
{
    Vector3d vel(0,0,0);
    Matrix<double, 6, 3> coe_temp = coe.block(6*m, 0, 6, 3);
    for(int i=0; i<3; i++)
    {
        for(int j=1; j<6; j++)
        {
            vel(i) += j * coe_temp.col(i)(j) * pow(t, j-1);
        }
    }
    return vel;
}

Vector3d TrajOpt::get_acc(double& dur)
{
    double t = 0;
    int t_rel = 0;
    int j = 0;
    if(dur <= t_total_)
    {
        for(int i=0; i<dur_.size(); i++)
        {
            t_rel += dur_[i];
            if(t_rel - dur > 0)
            {
                j = i;
                t = dur - t_rel + dur_[i];
                break;
            }
        }
    }
    Vector3d cur_acc = get_acc(coeff_, j, t);
    return cur_acc;
}

Vector3d TrajOpt::get_acc(const MatrixXd& coe, const int m, const double t)
{
    Vector3d acc(0,0,0);
    Matrix<double, 6, 3> coe_cur = coe.block(6*m, 0, 6, 3);
    for(int i=0; i<3; i++)
    {
        for(int j=2; j<6; j++)
        {
            acc(i) += j * (j-1) * coe_cur.col(i)(j) * pow(t, j-2);
        }
    }
    return acc;
}

void TrajOpt::traj_pub(const MatrixXd dp)
{
    //coeff_ = get_coeff(dp);

    traj_info_.t_total = t_total_;
    traj_info_.traj_pub = traj_start_;

    static int traj_Id = 0;
    traj_opt::TrajInfo traj;
    traj.ID = traj_Id++;
    traj.start_time = traj_start_;
    for(int i=0; i<coeff_.rows(); i++)
    {
        geometry_msgs::Point c;
        c.x = coeff_(i,0);
        c.y = coeff_(i,1);
        c.z = coeff_(i,2);
        traj.coeff.push_back(c);
    }
    for(int i=0; i<dur_.size(); i++)
    {
        traj.time.push_back(dur_[i]);
    }
    traj_pub_.publish(traj);
}

void TrajOpt::display_traj()
{
    if(!traj_vis_flag_) return;
    visualization_msgs::Marker traj_vis;
    traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.header.stamp = ros::Time::now();
    traj_vis.header.frame_id = "world";
    traj_vis.lifetime = ros::Duration();
    traj_vis.id = 0;

    traj_vis.scale.x = 0.3;
    traj_vis.scale.y = 0.3;
    traj_vis.scale.z = 0.3;

    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 1.0;
    traj_vis.color.a = 0.5;

    geometry_msgs::Point pt;
    Vector3d pos;
    for(int i=0; i<dur_.size(); i++)
    {
        for(double t=0; t<dur_[i]; t+=0.2)
        {
            pos = get_pos(coeff_, i, t);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            traj_vis.points.push_back(pt);
        }
    }
    traj_vis_pub_.publish(traj_vis);
    traj_vis_flag_ = false;
    //cout << "end_display_traj" << endl;
}

//not useful
void TrajOpt::display_va(const MatrixXd& coe)
{
    double t = 0.0; 
    ros::Time t1;
    Vector3d pt, vel3d, acc3d;
    int j = 0;
    double delta_t = 0.0;
    while(1)
    {
        double t_rel = 0.0;
        t1 = ros::Time::now();
        delta_t = (t1 - traj_start_).toSec(); 
        if(delta_t < t_total_)
        {
            for(int i=0; i<dur_.size(); i++)
            {
                t_rel += dur_[i];
                if(t_rel - delta_t > 0)
                {
                    j = i;
                    t = delta_t - t_rel + dur_[i];
                    break;
                }
            }
            pt = get_pos(coe, j, t);
            vel3d = get_vel(coe, j, t);
            acc3d = get_acc(coe, j, t);
            //publish_odom(pt, vel3d);
            double vel, acc;
            vel = vel3d.norm();
            acc = acc3d.norm();

            std_msgs::Float64 velMsg, accMsg;
            velMsg.data = vel;
            accMsg.data = acc;
            vel_pub_.publish(velMsg);
            acc_pub_.publish(accMsg);
        }
        else{break;}
    }
    cout << "processing over" << endl;
}





