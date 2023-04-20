#ifndef MAP_MANAGE_H
#define MAP_MANAGE_H

#include <vector>
#include <utility>
#include <limits> 
#include <algorithm>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

class MapManage
{
    public:
        //general function
        MapManage();
        ~MapManage();
        void init(ros::NodeHandle& nh);
        Vector3d get_grid_center(const Vector3d& pt);
        inline void axisId(Vector3i& id);
        inline Vector3d index2coord(const Vector3i& idx);
        inline Vector3i coord2index(const Vector3d& pt);
        inline int getVoxelNum(const int dim);

        //uesd for grid map
        void get_grid_map(const pcl::PointCloud<pcl::PointXYZ>& cloud);
        void resetBuffer(const Vector3d& rang_min, const Vector3d& range_max);
        void setObs(const double coord_x, const double coord_y, const double coord_z);
        inline bool has_gridmap();
        inline int voxel_id(const Vector3i temp_id);

        //uesd for sdf
        void get_sdf();
        pair<double, Vector3d> getSDFvalue_grad(const Vector3d& pos);
        void pub_sdf();
        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
        inline double get_SDFvalue(const Vector3i idx);
        inline bool has_sdf();
        void update_range();

        Vector3d odom_pt_;
        uint8_t* obs_buffer_;

    private:
        //general variables
        ros::Subscriber globalmap_sub_, grad_sub_;
        ros::Publisher PC_vis_pub_, gridmap_vis_pub_, ESDF_vis_pub_, sensor_range_pub_;
        
        Vector3d sensor_range_;
        Vector3i local_bound_max_, local_bound_min_;
        //used for grid map
        double x_size_, y_size_, z_size_; 
        Vector3d origin_;    //origin point
        Vector3d map_lower_, map_upper_; 
        int idx_x_, idx_y_, idx_z_, idx_yz_, idx_xyz_;    //grid number
        double grid_resolution_;
        bool get_gridmap_, get_sdf_;
        //uesd for sdf
        vector<double> dist_buffer_, dist_neg_buffer_;
        vector<double> temp_buffer1_, temp_buffer2_;
    public:
        typedef shared_ptr<MapManage> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline int MapManage::voxel_id(const Vector3i temp_id)
{
    return temp_id(0)*idx_y_*idx_z_ + temp_id(1)*idx_z_ + temp_id(2);
}

inline void MapManage::axisId(Vector3i& id)
{
    Vector3i temp;
    temp(0) = max(min(id(0), idx_x_-1), 0);
    temp(1) = max(min(id(1), idx_y_-1), 0);
    temp(2) = max(min(id(2), idx_z_-1), 0);
    id = temp;
}

inline Vector3d MapManage::index2coord(const Vector3i& idx)
{
    Vector3d pt;
    pt(0) = ((double)idx(0) + 0.5) * grid_resolution_ + map_lower_(0); 
    pt(1) = ((double)idx(1) + 0.5) * grid_resolution_ + map_lower_(1); 
    pt(2) = ((double)idx(2) + 0.5) * grid_resolution_ + map_lower_(2); 

    return pt;
}

inline Vector3i MapManage::coord2index(const Vector3d& pt)
{
    Vector3i idx;

    idx(0) = min( max( (int)((pt(0) - map_lower_(0))/grid_resolution_), 0), idx_x_-1);
    idx(1) = min( max( (int)((pt(1) - map_lower_(1))/grid_resolution_), 0), idx_y_-1);
    idx(2) = min( max( (int)((pt(2) - map_lower_(2))/grid_resolution_), 0), idx_z_-1);

    return idx;
}

inline int MapManage::getVoxelNum(const int dim)
{
    if(dim == 0){return idx_x_;}
    if(dim == 1){return idx_y_;}
    if(dim == 2){return idx_z_;}
}

inline bool MapManage::has_gridmap()
{
    return this->get_gridmap_;
}

inline double MapManage::get_SDFvalue(const Vector3i idx)
{
    return dist_buffer_[idx(0) * idx_yz_ + idx(1) * idx_z_ + idx(2)];
}

inline bool MapManage::has_sdf()
{
    return this->get_sdf_;
}
#endif   //MAP_MANAGE_H









