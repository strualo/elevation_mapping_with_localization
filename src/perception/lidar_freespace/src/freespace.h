#include <string>
#include <vector>
#include <ros/ros.h>
#include "ros/package.h"
#include <stdio.h>
#include <stack>
#include <math.h>
#include <time.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/console/time.h>

class FreeSpace
{
  public:
    FreeSpace(ros::NodeHandle nh,const float &polar_angle = 1., const float &polar_range_size = 0.15,
              const float &max_range = 50., const float &max_z_height = 0.7);
    ~FreeSpace() {}

    void Callback(const sensor_msgs::PointCloud2ConstPtr &points_msg);

  protected:
    float max_range_,
        max_z_height_,
        polar_angle_,
        polar_range_size_;
    int MSegs_, NBins_;
    float polar_min_rad_;
    int dilation_size_;
    bool polar_has_input_;
    bool is_sort_grids_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr polar_leave_cloud_ptr_;
    std::vector< std::vector< std::vector<pcl::PointXYZI> > > polar_save_grid_pts_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr air_cloud_ptr_;

  private:
    float computeXYDis(const pcl::PointXYZI &pt);
    float computBeta(const pcl::PointXYZI &pt);
    void GenPolarMat();
    void GenPolarSize(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);
    ros::Publisher map_pub;
    ros::Subscriber points_sub;
    int max_label_;
    int frames;
    cv::Mat label_mat_;
};
