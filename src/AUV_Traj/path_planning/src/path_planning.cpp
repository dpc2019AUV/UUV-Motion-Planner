#include "path_planning/path_planning.h"
PathPlanning::~PathPlanning()
{
    for(int i=0; i<node_list_.size(); i++)
    {
        delete node_list_[i];
    }
    for(int i=0; i<temp_exp_nodes_.size(); i++)
    {
        delete temp_exp_nodes_[i];
    }
}

void PathPlanning::init(ros::NodeHandle& nh)
{
    nh.param("path/x_size", x_size_, 60.0);
    nh.param("path/y_size", y_size_, 60.0);
    nh.param("path/z_size", z_size_, 20.0);
    nh.param("path/resolution", res_, 1.0);
    nh.param("path/motion_primitives_num", mp_num_, 5);  //9
    nh.param("path/allocate_num", allocate_num_, 1000);
    nh.param("dubins/step", step_, 1.0);
    nh.param("dubins/horizon_radius", radi_h_, 2.5);
    nh.param("dubins/vetical_radius", radi_v_, 3.0);

    nh.param("path/epsilon", epsilon_, 1.0);  //1.0
    nh.param("path/interpolation_dist", interpolation_dist_, -1.0);  //6.0

    used_node_num_ = 0;
    mp_visIDcount_ = 0;
    cur_vis_count_ = 0;
    root_vis_num_ = 0;

    origin_ << -x_size_/2, -y_size_/2, 0;
    map_upper_ << x_size_/2, y_size_/2, z_size_;
    x_num_ = (int)(x_size_/res_);
    y_num_ = (int)(y_size_/res_);
    z_num_ = (int)(z_size_/res_);

    // allocate_num_ = x_num_ * y_num_ * z_num_;
    node_list_.reserve(allocate_num_);
    node_list_.clear();
    for(int i=0; i<allocate_num_; i++)
    {
        node_list_.push_back(new Node);
    }
    temp_exp_nodes_.reserve(mp_num_);
    temp_exp_nodes_.clear();
    for (int i=0; i<mp_num_; i++)
    {
        temp_exp_nodes_.push_back(new Node);
    }
    mps_vis_pub_  = nh.advertise<visualization_msgs::MarkerArray>("/path_planning/exp_node_vis", 1);
    cur_node_pub_  = nh.advertise<visualization_msgs::Marker>("/path_planning/cur_node_vis", 1);
    path_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/path_planning/path_vis", 1);
    root_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/path_planning/root_vis", 1);
    path_search_pub = nh.advertise<visualization_msgs::Marker>("/path_planning/path_search_nodes", 1);
}

void PathPlanning::setmap(const MapManage::Ptr& env)
{
    map_ptr_ = env;
}

void PathPlanning::reset()
{
    used_node_num_ = 0;
    mp_visIDcount_ = 0;
    cur_vis_count_ = 0;
    root_vis_num_ = 0;
    for(int i=0; i<node_list_.size(); i++)
    {
        node_list_[i] -> idx_ = -1;
        node_list_[i] -> parent_ = nullptr;
        node_list_[i] -> state_ = UNEXPAND;
    }
    priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> empty_queue;   //need to reset
    open_set_.swap(empty_queue);
    path_.clear();
}



double PathPlanning::get_heur_value(const Vector3d start, const Vector3d angle_start, 
                                        const Vector3d goal, const Vector3d angle_goal)
{
    //horizon plane
    double ph0[3] = {start(0), start(1), angle_start(2)};
    double ph1[3] = {goal(0), goal(1), angle_goal(2)};
    Dubins dub_ptr_h = Dubins(ph0, ph1, radi_h_);
    double h_len = dub_ptr_h.dubins_path_length();

    //vertical plane
    double pv0[3] = {0.0, start(2), angle_start(1)};
    double pv1[3] = {h_len, goal(2), angle_goal(1)};
    Dubins dub_ptr_v = Dubins(pv0, pv1, radi_v_);
    double v_len = dub_ptr_v.dubins_path_length();

    return v_len;
}

void PathPlanning::get_exp_nodes(const NodePtr cur_ptr, const Vector3d goal_p, const Vector3d goal_rpy)
{
    Vector3d state;
    Vector3d cur_p = cur_ptr -> pt_;
    double yaw = cur_ptr -> rpy_(2);
    double pitch = cur_ptr -> rpy_(1);

    Matrix3d trans_z, trans_y;
    trans_z << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    trans_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
    for (int i=0; i<mp_num_; i++)
    {
        state << end_state[i][0], end_state[i][1], end_state[i][2];
        NodePtr ptr = temp_exp_nodes_[i];
        ptr -> pt_ = trans_z * trans_y * state + cur_p;
        ptr -> rpy_ << 0, pitch + end_state[i][4], yaw + end_state[i][3];
        ptr -> g_cost_ =  g_step[i] + cur_ptr -> g_cost_;
        ptr -> h_cost_ = get_heur_value(ptr -> pt_, ptr -> rpy_, goal_p, goal_rpy);
        ptr -> f_cost_ = ptr -> g_cost_ + ptr -> h_cost_;
        ptr -> parent_ = cur_ptr;

        while (ptr -> rpy_(2) > M_PI)
            ptr -> rpy_(2) -= 2*M_PI;
        while (ptr -> rpy_(2) < -M_PI)
            ptr -> rpy_(2) += 2*M_PI;

        if(ptr -> pt_(0) >= map_upper_(0) || ptr -> pt_(1) >= map_upper_(1) || ptr -> pt_(2) >= map_upper_(2))
            temp_exp_nodes_[i] -> parent_ = nullptr;
        if(ptr -> pt_(0) <= origin_(0) || ptr -> pt_(1) <= origin_(1) || ptr -> pt_(2) <= origin_(2))
            temp_exp_nodes_[i] -> parent_ = nullptr;
    }
}

void PathPlanning::coll_check(NodePtr& cur_ptr)
{
    Vector3d cur_p = cur_ptr -> pt_;
    double yaw = cur_ptr -> rpy_(2);
    double pitch = cur_ptr -> rpy_(1);
    Matrix3d trans_z, trans_y;
    trans_z << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    trans_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
    Vector3d state;
    for(int k=0; k<temp_exp_nodes_.size(); k++)
    {
        state << mid_state[k][0], mid_state[k][1], mid_state[k][2];
        Vector3d mid_s = trans_z * trans_y * state + cur_p;
        Vector3i id_m = map_ptr_ -> coord2index(mid_s);
        if (map_ptr_ -> obs_buffer_[id_m(0)*y_num_*z_num_ + id_m(1)*z_num_ + id_m(2)])
        { temp_exp_nodes_[k] -> parent_ = nullptr; continue; }

        state << end_state[k][0], end_state[k][1], end_state[k][2];
        Vector3d end_s = trans_z * trans_y * state + cur_p;
        Vector3i id_e = map_ptr_ -> coord2index(end_s);
        if (map_ptr_ -> obs_buffer_[id_e(0)*y_num_*z_num_ + id_e(1)*z_num_ + id_e(2)])
        { temp_exp_nodes_[k] -> parent_ = nullptr; }
    }
}

void PathPlanning::prune_nodes()
{
    for(int i=0; i<temp_exp_nodes_.size(); i++)
    {
        Vector3d p_candi = temp_exp_nodes_[i] -> pt_;
        Vector3i id_candi = map_ptr_ -> coord2index(p_candi);
        for(int j=i+1; j<temp_exp_nodes_.size(); j++)
        {
            Vector3d p_temp = temp_exp_nodes_[j] -> pt_;
            Vector3i id_temp = map_ptr_ -> coord2index(p_temp);
            if ( (id_candi(0) == id_temp(0)) && (id_candi(1) == id_temp(1)) 
                    && (id_candi(2) == id_temp(2)) && (temp_exp_nodes_[i] -> parent_ != NULL) && (temp_exp_nodes_[j] -> parent_ != NULL))
            {
                if (temp_exp_nodes_[i] -> f_cost_ < temp_exp_nodes_[j] -> f_cost_)
                {
                    temp_exp_nodes_[j] -> parent_ = nullptr;
                }
                else
                {
                    temp_exp_nodes_[i] -> parent_ = nullptr;
                    continue;
                }
            }
        }
    }
}

void PathPlanning::is_closed()
{
    for(int i=0; i<temp_exp_nodes_.size(); i++)
    {
        if (temp_exp_nodes_[i] -> parent_ != nullptr)
        {
            Vector3d pt = temp_exp_nodes_[i] -> pt_;
            Vector3i idx = map_ptr_ -> coord2index(pt);
            for(int j=0; j<node_list_.size(); j++)
            {
                Vector3d list_pt = node_list_[j] -> pt_;
                Vector3i list_idx = map_ptr_ -> coord2index(list_pt);
                bool in_closed = (node_list_[j] -> state_ == CLOSED);
                if( idx(0) == list_idx(0) && idx(1) == list_idx(1) 
                                && idx(2) == list_idx(2) && in_closed )
                {
                    temp_exp_nodes_[i] -> parent_ = nullptr;
                }
            }
        }
    }
}

void PathPlanning::nodes_after_filter()
{
    exp_nodes_.clear();
    for(int i=0; i<temp_exp_nodes_.size(); i++)
    {
        if(temp_exp_nodes_[i] -> parent_ != NULL)
        {
            exp_nodes_.push_back(temp_exp_nodes_[i]);
        }
    }
}

void PathPlanning::update_open()
{
    int k;
    for(int i=0; i<exp_nodes_.size(); i++)
    {
        bool in_open = false;
        Vector3d pt = exp_nodes_[i] -> pt_;
        Vector3i id = map_ptr_ -> coord2index(pt);
        for(int j = 0; j <= used_node_num_; j++)
        {
            Vector3d list_pt = node_list_[j] -> pt_;
            Vector3i list_id = map_ptr_ -> coord2index(list_pt);
            if(id(0) == list_id(0) && id(1) == list_id(1) && id(2) == list_id(2) 
                                                && node_list_[j] -> state_ == OPEN)
            {
                in_open = true;
                k = j;
                break;
            }
        }

        if ( in_open )
        {
            if (exp_nodes_[i] -> f_cost_ < node_list_[k] -> f_cost_)
            {
                NodePtr ptr = node_list_[k];
                ptr -> pt_ = exp_nodes_[i] -> pt_;
                ptr -> rpy_ = exp_nodes_[i] -> rpy_;
                ptr -> f_cost_ = exp_nodes_[i] -> f_cost_;
                ptr -> g_cost_ = exp_nodes_[i] -> g_cost_;
                ptr -> h_cost_ = exp_nodes_[i] -> h_cost_;
                ptr -> parent_ = exp_nodes_[i] -> parent_;
            }
        }
        else 
        {
            used_node_num_ += 1;
            //cout << "allocate_num_ = " << allocate_num_ <<", used_node_num_ = " << used_node_num_ << endl;
            node_list_[used_node_num_] -> pt_ = exp_nodes_[i] -> pt_;
            node_list_[used_node_num_] -> rpy_ = exp_nodes_[i] -> rpy_;

            node_list_[used_node_num_] -> f_cost_ = exp_nodes_[i] -> f_cost_;
            node_list_[used_node_num_] -> g_cost_ = exp_nodes_[i] -> g_cost_;

            node_list_[used_node_num_] -> h_cost_ = exp_nodes_[i] -> h_cost_;
            node_list_[used_node_num_] -> parent_ = exp_nodes_[i] -> parent_;

            node_list_[used_node_num_] -> state_ = OPEN;
            node_list_[used_node_num_] -> idx_ = used_node_num_;

            open_set_.push(node_list_[used_node_num_]);
        }
    }
}



void PathPlanning::path_retrieve(NodePtr& cur_ptr)
{
    NodePtr ptr = cur_ptr;
    vector<Vector3d> path;
    if(ptr -> parent_ == nullptr)
        return;
    while(ptr -> parent_ != nullptr)
    {
        path.push_back(ptr -> pt_);
        ptr = ptr -> parent_; 
    }
    reverse(path.begin(),path.end());    
    for(int i=0; i<path.size(); i++)
    {
        path_.push_back(path[i]);
    }
}

void PathPlanning::get_dubins_path(const vector< Matrix <double, 6, 1> >& dubins_path)
{
    for(int i=1; i<dubins_path.size(); i++)
    {
        Vector3d pt;
        pt << dubins_path[i](0), dubins_path[i](1), dubins_path[i](2);
        path_.push_back(pt);
    }
}



void PathPlanning::getMostDistantPoint(vector<Vector3d> segment, int &point_id, float &distance)
{
    Eigen::Vector3d front = segment.front();
    Eigen::Vector3d back = segment.back();

    point_id = -1;
    distance = -10.0;
    for(int i = 1; i < segment.size() - 1; i++)
    {
        float dist= 0;
        getDistanceToLine(front, back, segment[i], dist);
        if(dist > distance)
        {
            distance = dist;
            point_id = i;
        }
    }
}

void PathPlanning::breakSegment(vector<Vector3d> segment, int point_id,
                                      vector<Vector3d> &part1, vector<Vector3d> &part2)
{
    part1.insert(part1.begin(), segment.begin(), segment.begin() + point_id + 1);
    part2.insert(part2.begin(), segment.begin() + point_id, segment.end());
}

bool PathPlanning::allSegmentSimplified(vector<vector<Vector3d>> segments)
{
    for(int i= 0; i < segments.size(); i++)
    {
        if(segments[i].size() != 2)
        {
            return false;
        }
    }
    return true;
}

void PathPlanning::getDistanceToLine(Vector3d front, Vector3d back, Vector3d point, float &distance)
{
    Eigen::Vector4d line_dir(back(0) - front(0), back(1) - front(1), back(2) - front(2), 0);
    line_dir.normalize();
    Eigen::Vector4d p2p(point(0) - front(0), point(1) - front(1), point(2) - front(2), 0);

    distance= p2p.cross3(line_dir).squaredNorm();
    distance= sqrt(distance);
}

vector<Vector3d> PathPlanning::get_path()
{
    visualization_msgs::Marker search_nodes;
    search_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    search_nodes.action = visualization_msgs::Marker::ADD;
    search_nodes.header.stamp = ros::Time::now();
    search_nodes.header.frame_id = "world";
    search_nodes.ns = "search_path";
    search_nodes.lifetime = ros::Duration();
    search_nodes.id = 0;

    search_nodes.scale.x = 0.4;
    search_nodes.scale.y = 0.4;
    search_nodes.scale.z = 0.4;

    search_nodes.pose.orientation.w = 1.0;

    search_nodes.color.r = 0.0;
    search_nodes.color.g = 0.0;
    search_nodes.color.b = 1.0;
    search_nodes.color.a = 1.0;

    geometry_msgs::Point pt;
    for(int i=0; i<path_.size(); i++)
    {
        pt.x = path_[i](0);
        pt.y = path_[i](1);
        pt.z = path_[i](2);
        search_nodes.points.push_back(pt);
    }
    path_search_pub.publish(search_nodes);

    return path_;
}

void PathPlanning::interpolate_wp(vector<Vector3d>& path)
{
    vector<Vector3d> temp;
    for(int i=0; i<path.size()-1; i++)
    {
        Vector3d pt0 = path[i];
        Vector3d pt1 = path[i+1];
        double dist = (pt1 - pt0).norm();
        if(dist > interpolation_dist_)   
        {
            int seg = round(dist/interpolation_dist_);
            if(seg > 1)
            {
                for(int j = 0; j<seg; j++)
                {
                    Vector3d pt = pt0 + j*(pt1 - pt0)/seg;
                    temp.push_back(pt);
                }
            }
            else{temp.push_back(path[i]);}
        }
        else
        {
            temp.push_back(path[i]);
        }
    }
    temp.push_back(*(path.rbegin()));
    path = temp;
}

//lattice
void PathPlanning::display_exp_node(const NodePtr cur_ptr, const vector<NodePtr> exp_nodes_)
{
    //cur_node
    visualization_msgs::Marker cur_node_vis;
    cur_node_vis.type = visualization_msgs::Marker::SPHERE;
    cur_node_vis.header.stamp = ros::Time::now();
    cur_node_vis.header.frame_id = "world";

    cur_node_vanish = cur_node_vis;
    cur_node_vanish.action = visualization_msgs::Marker::DELETEALL;
    //cur_node_vanish.ns = "node_vanish";
    
    cur_node_vis.action = visualization_msgs::Marker::ADD;
    cur_node_vis.ns = "node_vis";
    cur_node_vis.id = cur_vis_count_;
    cur_node_vis.lifetime = ros::Duration();
    cur_node_vis.scale.x = 0.3;
    cur_node_vis.scale.y = 0.3;
    cur_node_vis.scale.z = 0.3;
    cur_node_vis.color.r = 0.5;
    cur_node_vis.color.g = 0.9;
    cur_node_vis.color.b = 0.0;
    cur_node_vis.color.a = 1.0;
    cur_node_vis.pose.orientation.w = 1.0;

    cur_node_vis.pose.position.x = cur_ptr -> pt_(0);
    cur_node_vis.pose.position.y = cur_ptr -> pt_(1);
    cur_node_vis.pose.position.z = cur_ptr -> pt_(2);

    cur_node_pub_.publish(cur_node_vis);

    //motion primitives
    visualization_msgs::MarkerArray mps_vis;
    mps_vis.markers.clear();
    visualization_msgs::Marker mp_vis;
    mp_vis.type = visualization_msgs::Marker::LINE_LIST;
    mp_vis.header.stamp = ros::Time::now();
    mp_vis.header.frame_id = "world";

    mp_vanish = mp_vis;
    //mp_vanish.ns = "mp_vanish";
    mp_vanish.action = visualization_msgs::Marker::DELETEALL;

    mp_vis.action = visualization_msgs::Marker::ADD;
    mp_vis.ns = "mp_vis";
    mp_vis.id = mp_visIDcount_;
    mp_vis.lifetime = ros::Duration();
    mp_vis.scale.x = 0.2;
    // mp_vis.scale.y = 0.2;
    // mp_vis.scale.z = 0.2;
    mp_vis.color.r = 1.0;
    mp_vis.color.g = 0.0;
    mp_vis.color.b = 0.0;
    mp_vis.color.a = 1.0;
    mp_vis.pose.orientation.w = 1.0;

    geometry_msgs::Point p_start, p_end;
    p_start.x = cur_ptr -> pt_(0);
    p_start.y = cur_ptr -> pt_(1);
    p_start.z = cur_ptr -> pt_(2);
    for(int i=0; i<exp_nodes_.size(); i++)
    { 
        p_end.x = exp_nodes_[i] -> pt_(0);
        p_end.y = exp_nodes_[i] -> pt_(1);
        p_end.z = exp_nodes_[i] -> pt_(2);

        mp_vis.points.push_back(p_start);
        mp_vis.points.push_back(p_end);
        mp_vanish.points.push_back(p_start);
        mp_vanish.points.push_back(p_end);

        mps_vis.markers.push_back(mp_vis);
        mps_vanish.markers.push_back(mp_vanish);
    }

    mps_vis_pub_.publish(mps_vis);
    mp_vis.points.clear();
    cur_vis_count_++;
    mp_visIDcount_++;
}

void PathPlanning::display_path(const vector<Vector3d>& path)
{
    visualization_msgs::Marker path_vis;
    path_vis.type = visualization_msgs::Marker::LINE_STRIP;
    path_vis.action = visualization_msgs::Marker::ADD;
    path_vis.header.stamp = ros::Time::now();
    path_vis.header.frame_id = "world";
    path_vis.lifetime = ros::Duration();
    path_vis.id = 0;

    path_vis.scale.x = 0.2;
    path_vis.scale.y = 0.2;
    path_vis.scale.z = 0.2;

    path_vis.pose.orientation.x = 0.0;
    path_vis.pose.orientation.y = 0.0;
    path_vis.pose.orientation.z = 0.0;
    path_vis.pose.orientation.w = 1.0;

    path_vis.color.r = 1.0;
    path_vis.color.g = 1.0;
    path_vis.color.b = 0.0;
    path_vis.color.a = 1.0;

    geometry_msgs::Point pt;
    for(int i=0; i<path.size(); i++)
    {
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        path_vis.points.push_back(pt);
    }
    path_vis_pub_.publish(path_vis);
}

void PathPlanning::display_root(const Vector3d root)
{
   // cout << "********** into display root **********" << endl;
    root_vis_num_++;
    visualization_msgs::Marker root_vis;
    root_vis.type = visualization_msgs::Marker::CUBE;
    root_vis.action = visualization_msgs::Marker::ADD;
    root_vis.header.stamp = ros::Time::now();
    root_vis.header.frame_id = "world";
    root_vis.ns = "root";
    root_vis.lifetime = ros::Duration();
    root_vis.id = root_vis_num_;

    root_vis.scale.x = 1;
    root_vis.scale.y = 1;
    root_vis.scale.z = 1;

    root_vis.pose.orientation.w = 1.0;

    root_vis.pose.position.x = root(0);
    root_vis.pose.position.y = root(1);
    root_vis.pose.position.z = root(2);

    root_vis.color.r = 1.0;
    root_vis.color.g = 0.0;
    root_vis.color.b = 0.0;
    root_vis.color.a = 0.3;

    root_vis_pub_.publish(root_vis);
    //cout << "has publish pose arrow" << endl;
}




