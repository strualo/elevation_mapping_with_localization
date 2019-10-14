#include "odom_imu/odom_imu.h"

//--------------------只使用IMU----------------
IMU::~IMU() 
{
    
}

bool IMU::init()
{
    ROS_INFO("Start init OdomImu.");

    first_imu_ = true;
    first_pose_ = true;
    tf_init_ = false;
    imu_pose_initial = false;
    imu_pose_re_initial = false;
    ndt_pose_lose = false;
    pnh_.param<double>("max_interval", param_max_interval_, 1.0);
    pnh_.param<double>("angle_vel_sensitive", param_angle_vel_sensitive_, 0.1);
    pnh_.param<double>("linear_vel_sensitive", param_linear_vel_sensitive_, 0.1);
    pnh_.param<std::string>("base_frame", param_base_frame_, std::string("/base_link"));
    pnh_.param<std::string>("odom_frame", param_odom_frame_, std::string("/odom"));
    pnh_.param<std::string>("sub_imu_topic", sub_imu_topic_, std::string("/imu/data"));
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(sub_imu_topic_, 200, boost::bind(&IMU::imuCB, this, _1));
    sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 10, boost::bind(&IMU::poseCB, this, _1));
    pub_imupose = nh_.advertise<geometry_msgs::PoseStamped>("/imu_current_pose",10);
    pub_imupath = nh_.advertise<nav_msgs::Path>("/debug/imu_path",10);
    pub_odom_imu_ = nh_.advertise<nav_msgs::Odometry>("/odomImu/odom_imu", 1);
    //pub_debug_vel_ = nh_.advertise<std_msgs::Float32>("/odom_imu/debug _vel", 1);

    ROS_INFO("End init OdomImu.");

    return true;
}

void IMU::poseCB(const geometry_msgs::PoseStampedConstPtr& msg)//给imu提供每个时间段的初始速度 并且给imu提供初始位置
{
    if(!imu_pose_initial)//给imu提供初始位置
    {
        ROS_INFO("odom_imu -> received current_pose");
        static int waitNum = 5; //丢弃前几次NDT定位的信息
        static int cnt = 0;
        if (cnt < waitNum) {
            cnt++;
            return;
        }

      imu_pose_initial = true;
      imu_init_pose.x = msg->pose.position.x;
      imu_init_pose.y = msg->pose.position.y;
      imu_init_pose.z = msg->pose.position.z;
      tf::Quaternion tmp_q1;
      tf::quaternionMsgToTF(msg->pose.orientation, tmp_q1);
      tf::Matrix3x3(tf::Quaternion(tmp_q1)).getEulerYPR(imu_init_pose.yaw, imu_init_pose.pitch, imu_init_pose.roll);
      ROS_INFO("initial imu pose:(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",imu_init_pose.x,imu_init_pose.y,imu_init_pose.z,
                                                   imu_init_pose.roll,imu_init_pose.pitch,imu_init_pose.yaw);
      pre_ndt_pose = imu_init_pose;                                        
    }

    if(imu_pose_re_initial)  //重置IMU当前位置 消除累计误差
    {
      imu_pose_re_initial = false;
      imu_current_pose_.x = msg->pose.position.x;
      imu_current_pose_.y = msg->pose.position.y;
      imu_current_pose_.z = msg->pose.position.z;
      tf::Quaternion tmp_q;
      tf::quaternionMsgToTF(msg->pose.orientation, tmp_q);
      tf::Matrix3x3(tf::Quaternion(tmp_q)).getEulerYPR(imu_init_pose.yaw, imu_init_pose.pitch, imu_init_pose.roll);
      ROS_INFO("re-initial imu pose:(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",imu_current_pose_.x,imu_current_pose_.y,imu_current_pose_.z,
                                                   imu_current_pose_.roll,imu_current_pose_.pitch,imu_current_pose_.yaw);
    }

    if (q.size() < 3) {   // std::queue<geometry_msgs::PoseStamped> q; 计算ndt速度
        q.push(*msg);
        return;
    }
    first_pose_ = false;
    q.pop();
    q.push(*msg);
    geometry_msgs::PoseStamped first_PoseStamped = q.front();
    geometry_msgs::PoseStamped last_PoseStamped = q.back();

    // if(std::sqrt(std::pow(last_PoseStamped.pose.position.x - pre_ndt_pose.x,2) + 
    //    std::pow(last_PoseStamped.pose.position.y - pre_ndt_pose.y,2) ) > 0.3) //|| 
    //    // last_PoseStamped.pose.orientation.yaw - pre_ndt_pose.yaw >0.2)
    // {
    //       ndt_pose_lose = true;  
    // }
    // else
    // {
    //       ndt_pose_lose = false;
    // }
    //std::cout<<"ndt_pose_lose = "<<ndt_pose_lose<<std::endl;

    pre_ndt_pose.x = msg->pose.position.x;
    pre_ndt_pose.y = msg->pose.position.y;
    pre_ndt_pose.z = 1.;
    tf::Quaternion tmp_q;
    tf::quaternionMsgToTF(msg->pose.orientation, tmp_q);
    tf::Matrix3x3(tf::Quaternion(tmp_q)).getEulerYPR(pre_ndt_pose.yaw, pre_ndt_pose.pitch, pre_ndt_pose.roll);

    double dis = std::sqrt(std::pow(first_PoseStamped.pose.position.x - last_PoseStamped.pose.position.x, 2) + std::pow(first_PoseStamped.pose.position.y - last_PoseStamped.pose.position.y, 2));

    ndt_vel.header = msg->header;
    ndt_vel.twist.linear.x = dis / (last_PoseStamped.header.stamp.toSec() - first_PoseStamped.header.stamp.toSec());
    ndt_vel.twist.linear.y = 0;
    ndt_vel.twist.linear.z = 0;
    // std_msgs::Float32 msg_vel;
    // msg_vel.data = ndt_vel.twist.linear.x;
    // pub_debug_vel_.publish(msg_vel);
    // ndt_vel.twist.angular.x = 0;
    // ndt_vel.twist.angular.y = 0;
    // ndt_vel.twist.angular.z = msg->cur_steer;   // TODO::Confirm--feedback.steer <-转化-> angular.z
    // nav_msgs::Odometry msg_odom_p;
    // msg_odom_p.header = msg->Header;
    // msg_odom_p.twist.twist = ndt_vel.twist;
    // pub_odom_.publish(msg_odom_p);
}

bool first_imu_current_pose=true;  //Imu初始化位姿

void IMU::imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(!imu_pose_initial)
    {
      //ROS_INFO("imu_pose_initial false!!");
      return;
    }

    if (first_imu_) 
    {
        odom_last_time = msg->header.stamp;  //记录下 收到一次imu信息的时间
        first_imu_ = false;
        pre_time_ = msg->header.stamp;
        ROS_INFO("Received first imu");
        return;
    } 
    else if (first_pose_) 
    {
        ROS_WARN("Have not received current_pose for initial, not publish odom");
        return;
    }

    double diff_time = (msg->header.stamp - pre_time_).toSec();
    if (diff_time > param_max_interval_) {
        ROS_WARN("Long time waiting for next imu msg. Igore this msg.");
        pre_time_ = msg->header.stamp;
        return;
    } 
    // else if ((msg->header.stamp - ndt_vel.header.stamp).toSec() > param_max_interval_) {
    //     ROS_WARN("Long time waiting for encoder msg update. Igore this msg.");
    //     return;
    // }
    pre_time_ = msg->header.stamp;


   if((msg->header.stamp - odom_last_time).toSec() > 1 )//当imu距离上次odom_last_time更新 工作1秒之后
   {
      imu_pose_re_initial=true;
      odom_last_time = msg->header.stamp;
   }
    // 进行一个低通滤波   round(x)返回x的四舍五入整数值。
    //double angle_vel_x = round(msg->angular_velocity.x / param_angle_vel_sensitive_) * param_angle_vel_sensitive_;
    //double angle_vel_y = round(msg->angular_velocity.y / param_angle_vel_sensitive_) * param_angle_vel_sensitive_;
    double angle_vel_z = round(msg->angular_velocity.z / param_angle_vel_sensitive_) * param_angle_vel_sensitive_;
    double linear_acc_x = round(msg->linear_acceleration.x / param_linear_vel_sensitive_) * param_linear_vel_sensitive_;
    double linear_acc_y = round(msg->linear_acceleration.y / param_linear_vel_sensitive_) * param_linear_vel_sensitive_;
    double linear_acc_z = 0;
    //std::cout<<angle_vel_z<<"  "<<linear_acc_x<<"  "<<std::endl;
    // linear_acc_y = 0.;
    // linear_acc_z = 0.;
    //double offset_roll = angle_vel_x * diff_time;
    //double offset_pitch = angle_vel_y * diff_time;
    //double offset_yaw = angle_vel_z * diff_time;
    // imu_current_pose_.roll += angle_vel_x * diff_time;
    // imu_current_pose_.pitch += angle_vel_y * diff_time;
    // double accX1 = linear_acc_x;
    // double accY1 = std::cos(imu_current_pose_.roll) * linear_acc_y - std::sin(imu_current_pose_.roll) * linear_acc_z;
    // double accZ1 = std::sin(imu_current_pose_.roll) * linear_acc_y + std::sin(imu_current_pose_.roll) * linear_acc_z;

    // double accX2 = std::sin(imu_current_pose_.pitch) * accZ1 + std::cos(imu_current_pose_.pitch) * accX1;
    // double accY2 = accY1;
    // double accZ2 = std::cos(imu_current_pose_.pitch) * accZ1 - std::cos(imu_current_pose_.pitch) * accX1;

    // double accX = std::cos(imu_current_pose_.yaw) * accX2 - std::sin(imu_current_pose_.yaw) * accY2;
    // double accY = std::sin(imu_current_pose_.yaw) * accX2 + std::cos(imu_current_pose_.yaw) * accY2;
    // double accZ = accZ2;
    // imu_current_pose_.x += current_vel_x_ * diff_time + accX * diff_time * diff_time / 2.0;
    // imu_current_pose_.y += current_vel_y_ * diff_time + accY * diff_time * diff_time / 2.0;
    // imu_current_pose_.z += current_vel_z_ * diff_time + accZ * diff_time * diff_time / 2.0;

    if(first_imu_current_pose)
    {
        first_imu_current_pose = false;
        imu_current_pose_.yaw =imu_init_pose.yaw;
        imu_current_pose_.x = imu_init_pose.x;
        imu_current_pose_.y = imu_init_pose.y;
        imu_current_pose_.z = imu_init_pose.z;
        //imu_vx0 = ndt_vel.twist.linear.x;
    }
    else
    {
    //std::cout<<"imu_vx0--------------"<<imu_vx0<<std::endl;
    //   std::cout<<"linear_acc_x--------------"<<linear_acc_x<<std::endl;
    //    std::cout<<"diff_time--------------"<<diff_time<<std::endl;
    //    std::cout<<"111--------------"<<imu_current_pose_.x <<std::endl;
       //imu_vx0 += linear_acc_x*diff_time;
        imu_current_pose_.yaw += angle_vel_z * diff_time;
        // imu_current_pose_.roll = 0.;wo
        // imu_current_pose_.pitch = 0.;
        //imu_current_pose_.x += ( imu_vx0*diff_time + 0.5*linear_acc_x*diff_time*diff_time) * std::cos(imu_current_pose_.yaw);
        //imu_current_pose_.y += ( imu_vx0*diff_time + 0.5*linear_acc_x*diff_time*diff_time) * std::sin(imu_current_pose_.yaw);
        imu_current_pose_.x += ndt_vel.twist.linear.x * std::cos(imu_current_pose_.yaw) * diff_time;
        imu_current_pose_.y += ndt_vel.twist.linear.x * std::sin(imu_current_pose_.yaw) * diff_time;
        imu_current_pose_.z = 1.;
    }
    
    // current_vel_x_ += accX * diff_time;
    // current_vel_y_ += accY * diff_time;
    // current_vel_z_ += accZ * diff_time;


    pose2GeometryPose(msg_imu_current_pose_.pose, imu_current_pose_);
    msg_imu_current_pose_.header.stamp=msg->header.stamp;
    msg_imu_current_pose_.header.frame_id=param_odom_frame_;
    pub_imupose.publish(msg_imu_current_pose_);  //发布IMU当前位置
    //pub_debug_imupath( );

    // if (!tf_init_) {
    //     try {
    //         tf_listener_.waitForTransform(param_base_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
    //         tf_listener_.lookupTransform(param_base_frame_, msg->header.frame_id, ros::Time(0), tf_btoi_);
    //     } catch (tf::TransformException& ex) {
    //         ROS_ERROR("Transform error in imuCB: %s", ex.what());
    //         return;
    //     }
    // }
    tf::Quaternion tmp_q;
    tmp_q.setRPY(0., 0., imu_current_pose_.yaw);
    tf::Transform transform2(tmp_q, tf::Vector3(imu_current_pose_.x, imu_current_pose_.y, imu_current_pose_.z));
    //transform odom->imu to odom->base
    //tf::Transform transform = transform2 * tf_btoi_.inverse();
    imu_tf_broadcaster_.sendTransform(tf::StampedTransform(transform2, msg->header.stamp, param_odom_frame_, param_base_frame_));

    // msg_odom_.header.stamp = msg->header.stamp;
    // msg_odom_.header.frame_id = param_odom_frame_;
    // msg_odom_.child_frame_id = param_base_frame_;
    // // tf::pointTFToMsg(transform.getOrigin(), msg_odom_.pose.pose.position);
    // // tf::quaternionTFToMsg(transform.getRotation(), msg_odom_.pose.pose.orientation);
    // msg_odom_.twist.twist.angular.x = angle_vel_x;
    // msg_odom_.twist.twist.angular.y = angle_vel_y;
    // msg_odom_.twist.twist.angular.z = angle_vel_z;
    // msg_odom_.twist.twist.linear.x = ndt_vel.twist.linear.x;
    // msg_odom_.twist.twist.linear.y = ndt_vel.twist.linear.y;
    // msg_odom_.twist.twist.linear.z = ndt_vel.twist.linear.z;
    // pub_odom_imu_.publish(msg_odom_);
}

void IMU::pub_debug_imupath( )
{

    geometry_msgs::PoseStamped p;

    
    p.pose.position.x = imu_current_pose_.x;
    p.pose.position.y = imu_current_pose_.y;
    p.pose.position.z = imu_current_pose_.z;

    Eigen::AngleAxisd roll_angle(imu_current_pose_.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(imu_current_pose_.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(imu_current_pose_.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = roll_angle * pitch_angle * yaw_angle;
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    debug_imupath.poses.push_back(p);
    pub_imupath.publish(debug_imupath);
}



