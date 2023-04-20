#include "map/map_manage.h"

MapManage::MapManage()
{
    get_gridmap_ = get_sdf_ = false;
}

MapManage::~MapManage()
{
    for(int i=0; i<idx_xyz_; i++)
    {
        delete [] obs_buffer_;
    }
}

void MapManage::init(ros::NodeHandle& nh)
{
    nh.param("map/grid_resolution", grid_resolution_, 1.0);
    nh.param("map/x_size", x_size_, 60.0);
    nh.param("map/y_size", y_size_, 60.0);
    nh.param("map/z_size", z_size_, 20.0);
    nh.param("map/sensor_range_x", sensor_range_(0), 15.0);
    nh.param("map/sensor_range_y", sensor_range_(1), 15.0);
    nh.param("map/sensor_range_z", sensor_range_(2), 3.0);
    
    map_lower_ << -x_size_/2, -y_size_/2, 0;
    map_upper_ << x_size_/2, y_size_/2, z_size_;
    origin_ = map_lower_;

    idx_x_ = (int)( x_size_ / grid_resolution_ );
    idx_y_ = (int)( y_size_ / grid_resolution_ );
    idx_z_ = (int)( z_size_ / grid_resolution_ );
    idx_yz_ = idx_y_ * idx_z_;
    idx_xyz_ = idx_x_ * idx_y_ * idx_z_;
    //set obstacle buffer
    obs_buffer_ = new uint8_t[idx_xyz_];
    memset(obs_buffer_, 0, idx_xyz_ * sizeof(uint8_t));
    dist_buffer_ = vector<double>(idx_xyz_, 10000);
    dist_neg_buffer_ = vector<double>(idx_xyz_, 10000);
    temp_buffer1_ = vector<double>(idx_xyz_, 0);
    temp_buffer2_ = vector<double>(idx_xyz_, 0);

    ESDF_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map/esdf_vis", 1);
    //PC_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/PC_vis", 1);
    gridmap_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map/gridmap_vis", 1);
    sensor_range_pub_ = nh.advertise<visualization_msgs::Marker>("map/sensor_range",1);

}

void MapManage::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    int x = static_cast<int>( (coord_x - map_lower_(0)) / grid_resolution_ );
    int y = static_cast<int>( (coord_y - map_lower_(1)) / grid_resolution_ );
    int z = static_cast<int>( (coord_z - map_lower_(2)) / grid_resolution_ );
    obs_buffer_[x *idx_yz_ + y * idx_z_ + z] = 1;
}

Vector3d MapManage::get_grid_center(const Vector3d& pt)
{
    Vector3i id = coord2index(pt);
    Vector3d grid_center = index2coord(id);
    return grid_center;
}

void MapManage::resetBuffer(const Vector3d& rang_min, const Vector3d& range_max)
{
    Vector3i min_id, max_id;
    min_id = coord2index(rang_min);
    max_id = coord2index(range_max);
    axisId(min_id);
    axisId(max_id);
    for(int x=min_id(0); x<=max_id(0); x++)
    {
        for(int y=min_id(1); y<=max_id(1); y++)
        {
            for(int z=min_id(2); z<=max_id(2); z++)
            {
                obs_buffer_[x *idx_yz_ + y * idx_z_ + z] = 0;
                dist_buffer_[x *idx_yz_ + y * idx_z_ + z] = 10000;
            }
        }
    }
}



void MapManage::update_range()
{
    Vector3d min_range, max_range;

    Vector3i cube_size = (local_bound_max_ - local_bound_min_)/2;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = 0;

    mk.pose.position.x = odom_pt_(0);
    mk.pose.position.y = odom_pt_(1);
    mk.pose.position.z = odom_pt_(2);

    mk.scale.x = cube_size(0);
    mk.scale.y = cube_size(1);
    mk.scale.z = cube_size(2);

    mk.color.a = 0.3;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    mk.pose.orientation.w = 1.0;
    sensor_range_pub_.publish(mk);
}



void MapManage::pub_sdf()
{
    double dist;
    sensor_msgs::PointCloud2 grad_vis;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;
    Vector3d pos;
    double z = 3.2;
    double resolution = 0.05;
    for (double x = map_lower_(0)+0.5 ; x < map_upper_(0)-0.5; x += resolution)
	{
		for (double y = map_lower_(1)+0.5 ; y < map_upper_(1)-0.5; y += resolution)
		{
            pos = Vector3d(x,y,z);
            pt.x = x;
            pt.y = y;
            pt.z = z; // + 0.01 * travelcost_map -> getSDFValue(pos);
            dist = getSDFvalue_grad(pos).first;
            pt.intensity = dist;
            cloud.points.push_back(pt);
		}
	}
    pcl::toROSMsg( cloud, grad_vis);
    grad_vis.header.frame_id = "world";
    ESDF_vis_pub_.publish(grad_vis);
}





