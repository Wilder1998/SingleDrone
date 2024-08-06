#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "FY/FyLogger.hpp"
#include "Utils/tictoc.hpp"

using std::string;
pcl::VoxelGrid<pcl::PointXYZ> voxelSampler_;
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulatedCloud_(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher pub_cloud_accumulated_, pub_cloud_fy_;

// param
int param_frame_pub_count_ = 1;
int param_frame_skip_ = 1;
double param_filter_size_ = 0.1;
double param_filter_ceil_ = 2.0;
double param_filter_floor_ = 0.0;
double param_filter_dist_ = 10.0;
bool param_rviz_vis_flag_ = true;
int param_filter_pc_size_each_pub_ = 100;
int param_filter_pub_pc_size_ = 1000;
int param_pub_freq_ = 10;

int frameCount_ = 0;
int pubCount_ = 0;
int skip_frameCount_ = 0;
bool filter_dist_flag_ = true;
bool have_odom_ = false;
bool have_pc_to_pub_ = false;
Eigen::Vector3d odom_p_{0, 0, 0};
std::vector<Eigen::Matrix3Xd> pc_matrix_vec_;

//------------------------------------------------------------------------------------------------------

void pc_cbk(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    if( skip_frameCount_ != param_frame_skip_ )
    {
        skip_frameCount_++;
        return;
    }
    skip_frameCount_ = 0;
    frameCount_++;

    // 积累点云
    TicToc timer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloudMsg, *in_cloud);
    *accumulatedCloud_ += *in_cloud;

    voxelSampler_.setInputCloud(accumulatedCloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxelSampler_.filter(*downsampledCloud);
    *accumulatedCloud_ = *downsampledCloud;
    double time_downsample = timer.toc();
    std::cout << "down sample time: " << time_downsample << "ms" << std::endl;

    if (frameCount_ == param_frame_pub_count_)
    {
        have_pc_to_pub_ = true;

        timer.tic();

        int pc_size = accumulatedCloud_->points.size();
        int filter_pc_size = 0;
        std::vector<int> filter_pc_index_vec;
        filter_pc_index_vec.clear();

        // filter ceil. floor and dist
        for( int i = 0; i < pc_size; i++ )
        {
            Eigen::Vector3d tp;
            tp[0] = accumulatedCloud_->points[i].x;
            tp[1] = accumulatedCloud_->points[i].y;
            tp[2] = accumulatedCloud_->points[i].z;
            if( tp[2] > param_filter_ceil_ )
                continue;
            if( tp[2] < param_filter_floor_ )
                continue;
            if( filter_dist_flag_ && !have_odom_ )
            {
                ROS_ERROR("using filter_dist but have NO odom!!!");
                return;
            }
            if( filter_dist_flag_ && have_odom_ )
            {
                if( (tp-odom_p_).norm() > param_filter_dist_ )
                    continue;
            }
            filter_pc_index_vec.push_back(i);
            filter_pc_size++;
        }

        // 抽选要求的数量的点
        Eigen::Matrix3Xd pc_matrix;
        pc_matrix.resize(3, param_filter_pub_pc_size_);
        if( param_filter_pub_pc_size_ > filter_pc_size )
        {
            for( int i = 0; i < filter_pc_size; i++ )
            {
                pc_matrix.col(i)[0] = accumulatedCloud_->points[filter_pc_index_vec[i]].x;
                pc_matrix.col(i)[1] = accumulatedCloud_->points[filter_pc_index_vec[i]].y;
                pc_matrix.col(i)[2] = accumulatedCloud_->points[filter_pc_index_vec[i]].z;
            }
            for( int i = filter_pc_size; i < param_filter_pub_pc_size_; i++ )
            {
                pc_matrix.col(i)[0] = 0.0;
                pc_matrix.col(i)[1] = 0.0;
                pc_matrix.col(i)[2] = 0.0;
            }
        }
        else
        {
            int skip = filter_pc_size / param_filter_pub_pc_size_ + 1;
            int i_pub = 0;
            for( int i = 0; i < filter_pc_size; i+=skip )
            {
                pc_matrix.col(i_pub)[0] = accumulatedCloud_->points[filter_pc_index_vec[i]].x;
                pc_matrix.col(i_pub)[1] = accumulatedCloud_->points[filter_pc_index_vec[i]].y;
                pc_matrix.col(i_pub)[2] = accumulatedCloud_->points[filter_pc_index_vec[i]].z;
                i_pub++;
            }
            for( int i = 1; i_pub < param_filter_pub_pc_size_; i+=skip )
            {
                pc_matrix.col(i_pub)[0] = accumulatedCloud_->points[filter_pc_index_vec[i]].x;
                pc_matrix.col(i_pub)[1] = accumulatedCloud_->points[filter_pc_index_vec[i]].y;
                pc_matrix.col(i_pub)[2] = accumulatedCloud_->points[filter_pc_index_vec[i]].z;
                i_pub++;
            }
        }
        
        // 按照每帧发布允许的数量分配
        for( int i = 0; i < param_pub_freq_; i++ )
        {
            for( int j = 0; j < param_filter_pc_size_each_pub_; j++ )
            {
                pc_matrix_vec_[i].col(j) = pc_matrix.col(i*param_filter_pc_size_each_pub_+j);
            }
        }

        if( param_rviz_vis_flag_ )
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_accumulated(new pcl::PointCloud<pcl::PointXYZ>);
            out_cloud_accumulated->points.resize(pc_matrix.cols());
            for( int i = 0; i < pc_matrix.cols(); i++ )
            {
                out_cloud_accumulated->points[i].x = pc_matrix.col(i)[0];
                out_cloud_accumulated->points[i].y = pc_matrix.col(i)[1];
                out_cloud_accumulated->points[i].z = pc_matrix.col(i)[2];
            }

            sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*out_cloud_accumulated, *msg);
            msg->header.frame_id = "world";  
            msg->header.stamp = ros::Time::now();
            pub_cloud_accumulated_.publish(msg);
        }

        double time_pub = timer.toc();
        std::cout << "pub all size: " << pc_matrix.cols()  << std::endl;
        std::cout << "pub all time: " << time_pub << "ms"  << std::endl;

        frameCount_ = 0;
        pubCount_ = 0;
        accumulatedCloud_->clear();
    }
}

void odom_cbk(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    have_odom_ = true;
    odom_p_[0] = odomMsg->pose.pose.position.x;
    odom_p_[1] = odomMsg->pose.pose.position.y;
    odom_p_[2] = odomMsg->pose.pose.position.z;
}

void timer_cbk(const ros::TimerEvent&)
{
    // 分批发布之前积累的合起来降采样后的点云
    if(have_pc_to_pub_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        out_cloud->points.resize(pc_matrix_vec_[pubCount_].cols());
        for( int i = 0; i < pc_matrix_vec_[pubCount_].cols(); i++ )
        {
            out_cloud->points[i].x = pc_matrix_vec_[pubCount_].col(i)[0];
            out_cloud->points[i].y = pc_matrix_vec_[pubCount_].col(i)[1];
            out_cloud->points[i].z = pc_matrix_vec_[pubCount_].col(i)[2];
        }

        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*out_cloud, *msg);
        msg->header.frame_id = "world";  
        msg->header.stamp = ros::Time::now();
        pub_cloud_fy_.publish(msg);

        fy_series::FyLogger::SEND_POINTS("wxx", pc_matrix_vec_[pubCount_]);
        pubCount_++;
    }

    if( pubCount_==param_pub_freq_ )
        have_pc_to_pub_ = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fy_series_publisher");
    ros::NodeHandle nh;

    pub_cloud_accumulated_ = nh.advertise<sensor_msgs::PointCloud2>("fy/cloud_accumulated", 1);
    pub_cloud_fy_ = nh.advertise<sensor_msgs::PointCloud2>("fy/cloud_fy", 1);

    int lidar_freq = 10;
    nh.param<int>("lidar_freq", lidar_freq, 1);
    nh.param<int>("frame_pub_count", param_frame_pub_count_, 1);
    nh.param<int>("frame_skip", param_frame_skip_, 1);
    nh.param<double>("filter_size",  param_filter_size_, 0.1);
    nh.param<double>("filter_ceil",  param_filter_ceil_, 2.0);
    nh.param<double>("filter_floor", param_filter_floor_, 0.0);
    nh.param<double>("filter_dist", param_filter_dist_, 10.0);
    nh.param<bool>("rviz_vis_flag", param_rviz_vis_flag_, true);
    nh.param<int>("filter_pc_size_each_pub", param_filter_pc_size_each_pub_, 100);
    nh.param<int>("pub_freq", param_pub_freq_, 10);
    // real pub freq
    param_pub_freq_ = (1.0 * param_pub_freq_) / ((1 + param_frame_skip_) * param_frame_pub_count_ / ( 1.0 * lidar_freq ));

    param_filter_pub_pc_size_ = param_pub_freq_ * param_filter_pc_size_each_pub_;

    pc_matrix_vec_.resize(param_pub_freq_);
    for( int i = 0; i < param_pub_freq_; i++ )
    {
        pc_matrix_vec_[i].resize(3, param_filter_pc_size_each_pub_);
    }
    voxelSampler_.setLeafSize(param_filter_size_, param_filter_size_, param_filter_size_);
    filter_dist_flag_ = param_filter_dist_ < 0.0 ? false : true;

    ros::Subscriber sub_pc = nh.subscribe("/cloud_registered", 5, pc_cbk);
    ros::Subscriber sub_odom = nh.subscribe("/Odometry", 5, odom_cbk);
    ros::Timer pub_timer = nh.createTimer(ros::Duration(1.0/param_pub_freq_), timer_cbk);

    ros::spin();

    return 0;
}