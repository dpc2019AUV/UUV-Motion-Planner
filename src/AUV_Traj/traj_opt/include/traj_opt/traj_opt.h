#ifndef TRAJ_OPT_H
#define TRAJ_OPT_H

//#define EIGEN_USE_MKL_ALL

#include <ros/ros.h>
#include <Eigen/Eigen>
//#include <nlopt.hpp>
#include <iostream>
#include <cmath>
#include <utility>
#include <cblas.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "traj_opt/TrajInfo.h"
#include "map/map_manage.h"
#include "traj_opt/lbfgs.hpp"

using namespace std;
using namespace Eigen;

const auto Factorial = [](int x){
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
};

struct TrajInfo
{
    ros::Time traj_pub;
    double t_total;
    vector<double> dur;
    MatrixXd coeff;
};

enum OPT_STATE {HORIZON, GOAL};

class TrajOpt
{
    public:
        TrajOpt();
        ~TrajOpt();
        void init(ros::NodeHandle& nh);
        void setmap(const MapManage::Ptr& env);
        void poly_param(const vector<Vector3d>& path, bool flag);
        void reset();
        void get_C(const int seg_num);
        void get_M(const int seg_num);
        void get_Q(const int seg_num);
        pair<MatrixXd, MatrixXd> init_guess(const Vector3d& odom_v, const Vector3d& goal_v, const Vector3d& odom_a, const Vector3d& goal_a);
        MatrixXd dp_init(const MatrixXd& df, const MatrixXd& dp);
        bool optimize(MatrixXd& dp, const MatrixXd& df, bool flag);
        static double costFunc(void *ptr, const VectorXd &x, VectorXd &grad);
        //static double costFunc(const vector<double> &x, vector<double> &grad, void *func_data);
        void getCostGrad(const VectorXd &x, double &cost, VectorXd &grad);
        void smooth(const MatrixXd& dp, double& cost, MatrixXd& grad);
        void collide(const MatrixXd& dp, double& cost, MatrixXd& grad);
        void dyn(const MatrixXd& dp, double& cost, MatrixXd& grad);
        void curv(const MatrixXd& dp, double& cost, MatrixXd& grad);
        void angle(const MatrixXd& dp, double& cost, MatrixXd& grad);
        void getWP(const MatrixXd& dp, const MatrixXd& df);
        MatrixXd get_coeff(const MatrixXd& dp);
        Vector3d get_pos(double& dur);
        Vector3d get_pos(const MatrixXd& coe, const int i, const double t);
        Vector3d get_vel(double& dur);
        Vector3d get_vel(const MatrixXd& coe, const int i, const double t);
        Vector3d get_acc(double& dur);
        Vector3d get_acc(const MatrixXd& coe, const int i, const double t);
        vector<double> get_dura();
        double get_time();
        void traj_pub(const MatrixXd dp);
        void display_traj();
        void display_va(const MatrixXd& coe);
        //void publish_odom(const Vector3d& pt, const Vector3d& vel);
        TrajInfo traj_info_; 
        vector<double> dur_;
        vector<Vector3d> wp_;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        ros::Publisher traj_vis_pub_, traj_pub_, vel_pub_, acc_pub_;
        ros::Time traj_start_;
        MapManage::Ptr map_ptr_;
        int order_, coeff_num_;
        double t_total_;
        double v_mean_, v_max_, a_max_, wh_max_, wv_max_, cur_max_, p_max_;
        double safe_dist_;
        bool traj_vis_flag_;
        MatrixXd Q_, M_, C_, R_, L_, coeff_;
        MatrixXd df_;   
        Matrix<double, 6, 6> V_;
        int df_num_;
        bool reach_goal_, only_collide_;
        OPT_STATE opt_state_;
    public:
        //typedef shared_ptr<TrajOpt> Ptr;

};
#endif  //TRAJ_OPT_H