#pragma once

#include <ros/ros.h>

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>

class road_space_detector
{
  private:

    nav_msgs::OccupancyGrid grid_map;
    float gridmap_width;   //grid map 的长宽  这里的长宽指的是像素点的数量！而不是实际距离 距离 = width或者height * resolution
    float gridmap_height;
    float resolution;
    float origion_x,origion_y; //地图最左下角的坐标 此时为距离坐标
    unsigned int map_data_index;  //map->data[]的索引，也就是具体的一个像素点的索引

    bool get_gridmap_flag;
    
    ros::ServiceClient get_gridmap_client; //获取grid map 服务(static_map)的 client
    ros::Subscriber sub_gridmap; //同时也可以订阅 /map topic
    ros::Subscriber sub_vlp_points;
    ros::Publisher pub_road_space_point;
    tf::TransformListener pose_tf_listener;
    tf::StampedTransform transform;
    Eigen::Matrix4f tf_matrix;

    pcl::PointCloud<pcl::PointXYZ> road_space_point;  //输出的 路面空间点云

  public:
    road_space_detector(ros::NodeHandle &nh);
    ~road_space_detector( );

    bool get_gridmap();
    void gridmap_cb(const nav_msgs::OccupancyGrid in_grid_map);
    unsigned int getIndex(float x,float y);
    
    unsigned char get_data(unsigned int index);
    unsigned char get_data_(unsigned int x,unsigned int y);//根据 像素坐标 读取像素值
    bool isBoundary(unsigned int index);
    std::vector<unsigned int> getFreeNeighbors(unsigned int index, int offset = 1);
    void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
};