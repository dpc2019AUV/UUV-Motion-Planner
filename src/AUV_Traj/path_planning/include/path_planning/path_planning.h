#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "map/map_manage.h"
#include "path_planning/dubins.h"

using namespace std;
using namespace Eigen;

//each motion primitives cost
const double g_step[9] = {1.5, 1.5*1.25, 1.5*1.25, 1.5*1.5, 1.5*1.5, 
                                1.5*2.0, 1.5*2.0, 1.5*2.0, 1.5*2.0};

//motion primitives state include middle and end instant
// {x, y, z, yaw, pitch}, solved with parameter: radius_h = 2.5m, radius_v = 3m
const double end_state[9][5] = {
    {1.5000,         0,         0,         0,         0},
    {1.2380,   -0.8470,         0,   -1.1352,         0},
    {1.2380,    0.8470,         0,    1.1352,         0},
    {1.3164,         0,    0.7191,         0,   -0.9591},
    {1.3164,         0,   -0.7191,         0,    0.9591},
    {1.0865,   -0.7433,    0.7191,   -1.2826,   -0.8346},
    {1.0865,    0.7433,    0.7191,    1.2826,   -0.8346},
    {1.0865,   -0.7433,   -0.7191,   -1.2826,    0.8346},
    {1.0865,    0.7433,   -0.7191,    1.2826,    0.8346} };

const double mid_state[9][5] = {
    {0.7500,         0,         0,         0,         0},
    {0.7165,   -0.2216,         0,   -0.5886,         0},
    {0.7165,    0.2216,         0,    0.5886,         0},
    {0.7267,         0,    0.1856,         0,   -0.4926},
    {0.7267,         0,   -0.1856,         0,    0.4926},
    {0.6942,   -0.2147,    0.1856,   -0.6070,   -0.4727},
    {0.6942,    0.2147,    0.1856,    0.6070,   -0.4727},
    {0.6942,   -0.2147,   -0.1856,   -0.6070,    0.4727},
    {0.6942,    0.2147,   -0.1856,    0.6070,    0.4727} };

enum Dubins_state {NO_PATH, SUB_PATH, REACH_GOAL};

enum State
{ OPEN = 1, CLOSED, UNEXPAND};

struct Node
{
    Vector3d pt_;
    Vector3d rpy_;
    double f_cost_, g_cost_, h_cost_;
    Node* parent_;
    State state_;
    int idx_;
    Node()
    {
        idx_ = -1;
        parent_ = nullptr;
        state_ = UNEXPAND;
    }
};
typedef Node* NodePtr; 

class NodeComparator
{
    public:
        bool operator()(NodePtr node1, NodePtr node2) { return node1->f_cost_ > node2->f_cost_; }   
};

class PathPlanning
{
    public:
        ~PathPlanning();
        void init(ros::NodeHandle& nh);
        void reset();
        void setmap(const MapManage::Ptr& env);
        void search(const Vector3d start, const Vector3d angle_start, 
                        const Vector3d goal, const Vector3d angle_goal);
        double get_heur_value(const Vector3d start, const Vector3d angle_start, 
                                const Vector3d goal, const Vector3d angle_goal);
        void get_exp_nodes(const NodePtr cur_ptr, const Vector3d goal_p, const Vector3d goal_rpy);
        void coll_check(NodePtr& cur_ptr);
        void prune_nodes();
        void is_closed();
        void nodes_after_filter();
        void update_open();
        bool dubins_expand(NodePtr cur_ptr, const Vector3d goal_p, const Vector3d goal_rpy, vector< Matrix <double, 6, 1> >& dubins_path);
        Dubins_state dubins_check(vector<Matrix <double, 6, 1> >& dubins_path, vector<Matrix <double, 6, 1>>::iterator& iter);
        void path_retrieve(NodePtr& cur_ptr);
        void get_dubins_path(const vector< Matrix <double, 6, 1> >& dubins_path);
        vector<Vector3d> get_path();
        void display_exp_node(const NodePtr cur_ptr, const vector<NodePtr> exp_nodes_);
        void display_root(const Vector3d root);
        void display_path(const vector<Vector3d>& path);

        void simplify_path(vector<Vector3d>& path);
        void getMostDistantPoint(vector<Vector3d> segment, int &point_id, float &distance);
        void breakSegment(vector<Vector3d> segment, int point_id,
                            vector<Vector3d> &part1, vector<Vector3d> &part2);
        bool allSegmentSimplified(vector<vector<Vector3d>> segments);
        void getDistanceToLine(Vector3d front, Vector3d back, Vector3d point, float &distance);
        void interpolate_wp(vector<Vector3d>& path);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        vector<NodePtr> node_list_;   //need to reset
        vector<NodePtr> temp_exp_nodes_, exp_nodes_;
        vector<Vector3d> path_;  //need to reset
        int allocate_num_, used_node_num_;
        int mp_num_;  //motion primitives num
        int mp_visIDcount_, cur_vis_count_, root_vis_num_;
        priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;   //need to reset
        //multimap<double, NodePtr> open_set_;

        ros::Publisher mps_vis_pub_, cur_node_pub_;
        ros::Publisher path_vis_pub_, root_vis_pub_, path_search_pub;

        //simplify path
        double epsilon_, interpolation_dist_;

        //dubins 
        double radi_h_, radi_v_;
        double step_;
        bool dubins_flag_;
        MapManage::Ptr map_ptr_;
        typedef Dubins* Dubins_Ptr;

        //map
        double x_size_, y_size_, z_size_;
        double res_;
        int x_num_, y_num_, z_num_;
        Vector3d origin_, map_upper_;

        //test
        visualization_msgs::Marker cur_node_vanish, mp_vanish;
        visualization_msgs::MarkerArray mps_vanish;

};

#endif //PATH_PLANNING_H