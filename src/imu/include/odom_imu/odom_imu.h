#ifndef __ODOM_IMU__
#define __ODOM_IMU__

#include "user_protocol.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
class IMU {
public:
    IMU(const ros::NodeHandle nh, const ros::NodeHandle pnh)
        : nh_(nh)
        , pnh_(pnh)
    {
    }
    ~IMU();
    bool init();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    tf::TransformBroadcaster imu_tf_broadcaster_;
    //tf::TransformListener tf_listener_;
    tf::StampedTransform tf_btoi_;
    bool tf_init_;

    ros::Publisher pub_odom_imu_;
    ros::Publisher pub_imupath;
    ros::Publisher pub_imupose;
    ros::Subscriber sub_pose_, sub_imu_;
    sensor_msgs::Imu msg_imu_;
    nav_msgs::Odometry msg_odom_;
    geometry_msgs::PoseStamped msg_imu_current_pose_;
    std::queue<geometry_msgs::PoseStamped> q;
    geometry_msgs::TwistStamped ndt_vel;

    ros::Time pre_time_;
    ros::Time odom_last_time;

    bool first_imu_;
    bool first_pose_;
    bool imu_pose_initial;
    bool imu_pose_re_initial;
    bool ndt_pose_lose;

    pose imu_init_pose;
    pose pre_ndt_pose;
    pose imu_current_pose_;
    double current_vel_x_;
    double current_vel_y_;
    double current_vel_z_;
    double imu_vx0;
    double param_max_interval_;
    double param_angle_vel_sensitive_;
    double param_linear_vel_sensitive_;
    double imu_vel=0;
    std::string param_base_frame_;
    std::string param_odom_frame_;
    std::string sub_imu_topic_;

    ros::Publisher pub_debug_vel_;
    nav_msgs::Path debug_imupath;

    void imuCB(const sensor_msgs::Imu::ConstPtr& msg);

    void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
    void pub_debug_imupath( );

};


#endif