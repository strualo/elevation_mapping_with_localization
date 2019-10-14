#include "freespace.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <pcl/console/time.h>

FreeSpace::FreeSpace(ros::NodeHandle nh, const float &polar_angle, const float &polar_range_size,
                     const float &max_range, const float &max_z_height)
{

    polar_angle_ = polar_angle;
    polar_range_size_ = polar_range_size;
    max_range_ = max_range;
    max_z_height_ = max_z_height;

    polar_min_rad_ = (polar_angle_ / 180.) * M_PI;
    MSegs_ = (2 * M_PI + 0.001) / polar_min_rad_;
    NBins_ = (max_range_ + 0.001) / polar_range_size_;

    polar_leave_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    polar_save_grid_pts_.resize(MSegs_);
    for (int j = 0; j < MSegs_; ++j)
    {
        polar_save_grid_pts_[j].resize(NBins_);
    }
    air_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    label_mat_ = cv::Mat::zeros(MSegs_,NBins_,CV_8U);
    dilation_size_ = 1;

    points_sub = nh.subscribe("/ray_filter/velodyne_points_filtered", 10, &FreeSpace::Callback,(FreeSpace *)this);

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("lidar_freespace_map",10);
    frames = 0;
    std::cout<<"the msg"<<std::endl;
}



void FreeSpace::GenPolarSize(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr)

{
     if (in_cloud_ptr->empty())
    {
        std::cout << "the input cloud is empty!!" << std::endl;
    }

    for (int j = 0; j < MSegs_; ++j)
    {
        for (int i = 0; i < NBins_; ++i)
        {
            polar_save_grid_pts_[j][i].clear();
        }
    }
    polar_leave_cloud_ptr_->clear();
    pcl::PointXYZI tmp_pt;
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        tmp_pt = in_cloud_ptr->points[i];
        if (isnan(tmp_pt.x) || isnan(tmp_pt.y) || isnan(tmp_pt.z) || isnan(tmp_pt.intensity))
            continue;
        float tmp_range = computeXYDis(tmp_pt); //计算点的距离
        if (tmp_range > max_range_)
        {
            polar_leave_cloud_ptr_->push_back(tmp_pt);
            continue;
        }
        if (tmp_pt.z > max_z_height_)
        {
            continue;
        }
        
        float tmp_beta = computBeta(tmp_pt);
        
        int x_index = tmp_beta / polar_min_rad_;
        int y_index = tmp_range / polar_range_size_;

        if(x_index==MSegs_)
            x_index--;
        if(y_index==NBins_)
            y_index--;
        polar_save_grid_pts_[x_index][y_index].push_back(tmp_pt);
     
    }

    polar_has_input_ = true;
}

bool computeZ(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
{
    return pt1.z < pt2.z;
}

float FreeSpace::computBeta(const pcl::PointXYZI &pt)  //计算角度
{
    float beta = atan2(pt.y, pt.x);
    beta = beta < 0. ? beta += 2 * M_PI : beta;
    beta = beta > (2 * M_PI) ? beta -= 2 * M_PI : beta;
    return beta;
}

float FreeSpace::computeXYDis(const pcl::PointXYZI &pt)
{
    return sqrtf(powf(pt.x, 2.) + powf(pt.y, 2.));
}

void FreeSpace::GenPolarMat()
{
    label_mat_ = cv::Mat::ones(MSegs_, NBins_, CV_8U);
    for (int j = 0; j < polar_save_grid_pts_.size(); ++j)
    {
        bool find_obstacle = false;
        for (int i = 0; i < polar_save_grid_pts_[j].size(); ++i)
        {
            int tmp_size = polar_save_grid_pts_[j][i].size();
            if (tmp_size <= 0)
            {
                label_mat_.at<uchar>(j, i) = 0;
                continue;
            }
            std::sort(polar_save_grid_pts_[j][i].begin(), polar_save_grid_pts_[j][i].end(), computeZ);
            if (polar_save_grid_pts_[j][i][tmp_size - 1].z < -1.5&& (polar_save_grid_pts_[j][i][tmp_size - 1].z - polar_save_grid_pts_[j][i][0].z) < 0.3)
            {
                label_mat_.at<uchar>(j, i) = 0;
            }
            else
            {
                break;
            }
        }
    }
     cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * 1 + 1, 2 * 1 + 1),
                                                cv::Point(1, 1));
    cv::dilate(label_mat_, label_mat_, element);
}

void FreeSpace::Callback(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{
    pcl::console::TicToc tt;//PCL 记录时间长度
    tt.tic( );
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*points_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_raw);
    GenPolarSize(cloud_raw);
    GenPolarMat();
    nav_msgs::MapMetaData info;
    info.resolution = 0.1;
    info.width = 500;
    info.height = 500;
    info.origin.position.x = -25.;
    info.origin.position.y = -25.;
    info.origin.position.z = -0.;
    info.origin.orientation.w = 1;
    nav_msgs::OccupancyGrid *newGrid(new nav_msgs::OccupancyGrid);
    newGrid->data.resize(info.width*info.height);
    // std::cout<<"the cost time is "<< tt.toc() << std::endl;
    for (unsigned int x = 0; x < info.width; x++)
    {
        for (unsigned int y = 0; y < info.height; y++)
        {
            newGrid->data[y * info.width + x] = 99;
             pcl::PointXYZI tmp_pt;
             tmp_pt.x = (x + 0.5) * 0.2-50;
             tmp_pt.y = (y + 0.5) * 0.2-50;
             tmp_pt.z = 0.;
             float tmp_range = computeXYDis(tmp_pt);
             float tmp_beta = computBeta(tmp_pt);
             int x_index = tmp_beta / polar_min_rad_;
             int y_index = tmp_range / polar_range_size_;
             if (x_index == MSegs_)
                 x_index--;
             if (y_index == NBins_)
                 y_index--;
            if(x_index>=MSegs_||y_index>=NBins_)
            {
                continue;
            }
            if(label_mat_.at<uchar>(x_index,y_index)==0)
            {
                  newGrid->data[y * info.width + x] = 20;  //设置为可行驶
            }

        }
    }
    newGrid->header = points_msg->header;
    newGrid->info = info;
    map_pub.publish(*newGrid);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "freespace");
    ros::NodeHandle nh;
    ros::NodeHandle pri_nh("~");
    float planefit_thre;
    std::string lidar_topic;
    pri_nh.getParam("planefit_thre", planefit_thre);
    pri_nh.getParam("lidar_topic",lidar_topic);
    FreeSpace freespacer(nh,1.,0.15,50.,1);
    
    ros::spin();
    return 0;
}
