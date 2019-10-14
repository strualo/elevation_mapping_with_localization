#include "road_space_detector.h"

road_space_detector::road_space_detector(ros::NodeHandle &nh)
{
    get_gridmap_flag = false;

    std::string service_name;
    nh.param<std::string>("map_service_name",service_name,std::string("static_map"));
      //创建了一个serviceClient mGetMapClient与已经创建的service（serviceName）进行通信
    get_gridmap_client = nh.serviceClient<nav_msgs::GetMap>(service_name);
    //std::cout<<"map_service_name:  "<<service_name<<std::endl;

    //sub_gridmap = nh.subscribe("/map",10,&road_space_detector::gridmap_cb,this);//同时也可以订阅/map话题

    sub_vlp_points = nh.subscribe("/ray_filter/velodyne_points_filtered",10,&road_space_detector::point_cb,this);
    pub_road_space_point = nh.advertise<sensor_msgs::PointCloud2>("/road_space_points",10);
}

road_space_detector::~road_space_detector( )
{

}


bool road_space_detector::get_gridmap( )
{
    if(get_gridmap_flag)
      return true;

    if(!get_gridmap_client.isValid())
    {
        ROS_ERROR("GetMap-Client is invalid!");
        return false;
    }

    nav_msgs::GetMap srv;
    if(!get_gridmap_client.call(srv))
    {
        ROS_INFO("Could not get a map.");
        return false;
    }

    grid_map = srv.response.map;
    resolution = grid_map.info.resolution;
    origion_x = grid_map.info.origin.position.x;//地图最左下角的坐标 此时为距离坐标！
    origion_y = grid_map.info.origin.position.y;

    gridmap_width = grid_map.info.width;
    gridmap_height = grid_map.info.height;
    get_gridmap_flag = true;  
    ROS_INFO("Success get grid map");
    return true;
}

// void road_space_detector::gridmap_cb(const nav_msgs::OccupancyGrid in_grid_map)
// {

// }

//输入为点云在点云地图中的位置 ,单位为 米!!  返回grid map.data 中像素的索引
unsigned int road_space_detector::getIndex(float x,float y)
{
    unsigned int grid_map_x,grid_map_y; //grid map 上的像素坐标

    grid_map_x = int( (x - origion_x)/resolution);
    grid_map_y = int( (y - origion_y)/resolution);
    if(grid_map_x>=gridmap_width || grid_map_y>=gridmap_height)
        return false;
    
    return grid_map_y*gridmap_width + grid_map_x;

}

//根据索引读取像素值
unsigned char road_space_detector::get_data(unsigned int index)
{
    if(index < gridmap_width*gridmap_height)
    {
        return grid_map.data[index]; //0  100
    }
    else //超出地图范围的点
        return 0;
}

//根据 像素坐标 读取像素值
unsigned char road_space_detector::get_data_(unsigned int x,unsigned int y)
{
	if(x < 0 ||x >= (int)gridmap_width || y < 0 || y >= (int)gridmap_height)
		return 100; //超出地图范围的点
	else
		return grid_map.data[y*gridmap_width + x];
}

//判断是否是边界点
bool road_space_detector::isBoundary(unsigned int index)
{
	int index_y = index / (int)gridmap_width;
	int index_x = index % (int)gridmap_width;
		
	if(get_data_(index_x-1, index_y-1) == 0) return true;
	if(get_data_(index_x-1, index_y  ) == 0) return true;
	if(get_data_(index_x-1, index_y+1) == 0) return true;
	if(get_data_(index_x  , index_y-1) == 0) return true;
	if(get_data_(index_x  , index_y+1) == 0) return true;
	if(get_data_(index_x+1, index_y-1) == 0) return true;
	if(get_data_(index_x+1, index_y  ) == 0) return true;
	if(get_data_(index_x+1, index_y+1) == 0) return true;
		
	return false;
}

// 计算黑色点 的 白色邻居点
std::vector<unsigned int> road_space_detector::getFreeNeighbors(unsigned int index, int offset )
{
	std::vector<unsigned int> neighbors;
		
	if(offset < 0) offset *= -1;
	int y = index / (int)gridmap_width;
	int x = index % (int)gridmap_height;
	
	for(int i = -offset; i <= offset; i++)
		for(int j = -offset; j <= offset; j++)
        {
            if(get_data_(x+i, y+j) == 0)
                neighbors.push_back(index);
        }
					
	return neighbors;
}

void road_space_detector::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before_tf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_tf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<bool> is_black;
    pose_tf_listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(1 * 10), ros::Duration(1 / 3));
    pose_tf_listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
    Eigen::Translation3f translation(transform.getOrigin().getX(),transform.getOrigin().getY(),0);
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_z(yaw,Eigen::Vector3f::UnitZ());
    tf_matrix = (translation*rot_z).matrix();


    pcl::fromROSMsg(*in_cloud_ptr,*cloud_before_tf);
    pcl::transformPointCloud(*cloud_before_tf, *cloud_after_tf, tf_matrix);

    // pcl::PassThrough<pcl::PointXYZ> pass; //过滤高于设定值的点
    // pass.setInputCloud(cloud_after_tf);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-10,1.5);
    // pass.filter(*cloud_after_tf);


    for(unsigned int i=0; i<cloud_after_tf->size(); i++)
    {
       unsigned int index_ = getIndex( cloud_after_tf->points[i].x, cloud_after_tf->points[i].y );
        //    if( get_data(index_)== 100 )//静态障碍物
        //        is_black.push_back(true);
        //    else
        //        is_black.push_back(false); 
     if( get_data(index_) > 50 )//静态障碍物
       { 
          //ROS_INFO("aaaaaaaaaaaaaaaaaaaaaaaaaaa");
          //cloud_after_tf->erase( cloud_after_tf->begin() + i );
          cloud_after_tf->points[i].x = 0;
          cloud_after_tf->points[i].y = 0;
          cloud_after_tf->points[i].z = 0;
        }
    }

    pcl::transformPointCloud(*cloud_after_tf, *cloud_remove_obstacles, tf_matrix.inverse());

   sensor_msgs::PointCloud2 cloud_after_filtered_obstacles;
   pcl::toROSMsg(*cloud_remove_obstacles , cloud_after_filtered_obstacles);
   cloud_after_filtered_obstacles.header.stamp = ros::Time::now();
   cloud_after_filtered_obstacles.header.frame_id = "/velodyne";
   pub_road_space_point.publish( cloud_after_filtered_obstacles );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "road_space_detector");
    ros::NodeHandle nh("~");
    road_space_detector detector(nh);
    while( !detector.get_gridmap() );

    ros::spin();
    return 0;
}
