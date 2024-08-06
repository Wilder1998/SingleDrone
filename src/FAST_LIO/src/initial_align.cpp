#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include "Utils/tictoc.hpp"

#include <ros/ros.h>

// param
int param_accumulated_scan_ = 10;
int param_max_iterations_ = 0;
double param_map_size_ = 50;
int param_function_value_ = 1;
double param_voxelgrid_filter_size_;

string initial_map_pcd_name_ = "initial_map.pcd";
Eigen::Vector3d      Init_Body_pos_;
Eigen::Matrix3d      Init_Body_rot_by_align_gravity_;
Eigen::Vector3d Lidar_wrt_Body_T;
Eigen::Matrix3d Lidar_wrt_Body_R;

bool receive_source_ = false;
bool imu_init_ready_ = false;
int imu_count_for_align_gravity_ = 50;
int scan_count_ = 0;
int imu_count_ = 0;
Eigen::Vector3d accumulated_acc_{0, 0, 0};

Eigen::Matrix4f incrementaL_pose_ = Eigen::Matrix4f::Identity();
Eigen::Matrix4f result_pose_ = Eigen::Matrix4f::Identity();

// target 是地图，source 是点云
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_raw_(new pcl::PointCloud<pcl::PointXYZ>());

shared_ptr<Preprocess> p_pre_(new Preprocess());

//------------------------------------------------------------------------------------------------------

void Eigen_to_PCL(const Eigen::Vector3d &Eigen_tp, pcl::PointXYZ &pcl_tp)
{
    pcl_tp.x = Eigen_tp[0];
    pcl_tp.y = Eigen_tp[1];
    pcl_tp.z = Eigen_tp[2];
}

void PCL_to_Eigen(const pcl::PointXYZ &pcl_tp, Eigen::Vector3d &Eigen_tp)
{
    Eigen_tp[0] = pcl_tp.x;
    Eigen_tp[1] = pcl_tp.y;
    Eigen_tp[2] = pcl_tp.z;
}

void M4_to_RT(Eigen::Matrix4f pose, Eigen::Vector3d &pos, Eigen::Matrix3d &rot)
{
    pos = pose.block<3,1>(0,3).cast<double>();
    rot = pose.block<3,3>(0,0).cast<double>();
}

void RT_to_M4(Eigen::Vector3d pos, Eigen::Matrix3d rot, Eigen::Matrix4f &pose)
{
    pose.block<3,1>(0,3) = pos.cast<float>();
    pose.block<3,3>(0,0) = rot.cast<float>();
}

void multiM4matrix(Eigen::Matrix4f &source,Eigen::Matrix4f &poses_pub)
{
    // poses_pub.block<3,3>(0,0) = source.block<3,3>(0,0)*poses_pub.block<3,3>(0,0);
    // poses_pub.block<3,1>(0,3)(0) = poses_pub.block<3,1>(0,3)(0) + source.block<3,1>(0,3)(0);
    // poses_pub.block<3,1>(0,3)(1) = poses_pub.block<3,1>(0,3)(1) + source.block<3,1>(0,3)(1);
    // poses_pub.block<3,1>(0,3)(2) = poses_pub.block<3,1>(0,3)(2) + source.block<3,1>(0,3)(2);   

    poses_pub = source * poses_pub;
}

void translate_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3d pos, Eigen::Matrix3d rot)
{
    for( int i = 0; i < cloud->points.size(); i++ )
    {
        Eigen::Vector3d tp;
        PCL_to_Eigen(cloud->points[i], tp);
        tp = rot * tp + pos;
        Eigen_to_PCL(tp, cloud->points[i]);
    }
}

void translate_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f pose)
{
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
    M4_to_RT(pose, pos, rot);
    translate_cloud(cloud, pos, rot);
}

void publish_frame_world(const ros::Publisher & publisher)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*source_cloud_, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "world";
    publisher.publish(laserCloudmsg);
}

void publish_frame_world(const ros::Publisher & publisher, Eigen::Vector3d pos, Eigen::Matrix3d rot)
{
    static bool translate_flag = false;
    if( !translate_flag )
    {
        translate_cloud(source_cloud_raw_, pos, rot);
    }
    translate_flag = true;

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*source_cloud_raw_, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "world";
    publisher.publish(laserCloudmsg);
}

// benchmark for fast_gicp registration methods
template <typename Registration>
void test(Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& source,int iteration_num) 
{
    TicToc timer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < iteration_num; i++) {
        // reg.swapSourceAndTarget();
        // reg.clearSource();
        reg.setInputTarget(target);
        reg.setInputSource(source);
        reg.align(*aligned);
        incrementaL_pose_ = reg.getFinalTransformation();

        translate_cloud(source, incrementaL_pose_);

        multiM4matrix(incrementaL_pose_, result_pose_);
        std::cout <<"-----------------------------------" << std::endl;
        std::cout << "\033[0;32miteration: " << "\033[0m" << i << std::endl;
        std::cout << "\033[0;32mrelative_pos: " << "\033[0m" << std::endl;
        std::cout << incrementaL_pose_.block<3,1>(0,3).transpose() << std::endl;
        std::cout << "\033[0;32mrelative_rot: " << "\033[0m" << std::endl;
        std::cout << incrementaL_pose_.block<3,3>(0,0).cast<double>()<<std::endl;
    }
    double time = timer.toc();
    std::cout <<"=======================================" << std::endl;
    std::cout << "\033[0;32mICP time: " << "\033[0m" << time << " ms" << std::endl;
    std::cout << "\033[0;32mrelative_pos: " << "\033[0m" << std::endl;
    std::cout << result_pose_.block<3,1>(0,3).transpose() << std::endl;
    std::cout << "\033[0;32mrelative_rot: " << "\033[0m" << std::endl;
    std::cout << result_pose_.block<3,3>(0,0).cast<double>()<<std::endl;
}

void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg) 
{
    if(receive_source_)
        return;

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre_->process(msg, ptr);
    
    scan_count_++;
    if(scan_count_ == param_accumulated_scan_)
    {
        receive_source_ = true;
    }

    for (const auto& pt : ptr->points) 
    {
        pcl::PointXYZ xyz_point;
        xyz_point.x = pt.x;
        xyz_point.y = pt.y;
        xyz_point.z = pt.z;
        source_cloud_->points.push_back(xyz_point);
    }
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    if(receive_source_)
        return;

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre_->process(msg, ptr);
    
    scan_count_++;
    if(scan_count_ == param_accumulated_scan_)
    {
        receive_source_ = true;
    }

    for (const auto& pt : ptr->points) 
    {
        pcl::PointXYZ xyz_point;
        xyz_point.x = pt.x;
        xyz_point.y = pt.y;
        xyz_point.z = pt.z;
        source_cloud_->points.push_back(xyz_point);
    }
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    if(imu_init_ready_)
        return;

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    Eigen::Vector3d acc{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    acc = Lidar_wrt_Body_R * acc;
    accumulated_acc_ += acc;

    imu_count_ ++;
    if(imu_count_ == imu_count_for_align_gravity_)
    {
        imu_init_ready_ = true;
        accumulated_acc_.normalize();
        std::cout<< accumulated_acc_.transpose() << std::endl;
        Init_Body_rot_by_align_gravity_ = Eigen::Quaterniond::FromTwoVectors(accumulated_acc_, Eigen::Vector3d(0, 0, 1));
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "initial_align");
    ros::NodeHandle nh;
    string  lid_topic, imu_topic;
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");

    nh.param<double>("preprocess/blind", p_pre_->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre_->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre_->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre_->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre_->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre_->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre_->feature_enabled, false);
    cout<<"p_pre_->lidar_type "<<p_pre_->lidar_type<<endl;

    nh.param<double>("wxx/initial_align/Init_Body_pos_x", Init_Body_pos_[0], 0.0);
    nh.param<double>("wxx/initial_align/Init_Body_pos_y", Init_Body_pos_[1], 0.0);
    nh.param<double>("wxx/initial_align/Init_Body_pos_z", Init_Body_pos_[2], 0.0);
    nh.param<string>("wxx/initial_map_pcd_name", initial_map_pcd_name_, "initial_map.pcd");
    vector<double> Lidar_wrt_Body_T_vec(3, 0.0);
    vector<double> Lidar_wrt_Body_R_vec(9, 0.0);
    nh.param<vector<double>>("wxx/Lidar_wrt_Body_T", Lidar_wrt_Body_T_vec, vector<double>());
    nh.param<vector<double>>("wxx/Lidar_wrt_Body_R", Lidar_wrt_Body_R_vec, vector<double>());
    Lidar_wrt_Body_T << Lidar_wrt_Body_T_vec[0], Lidar_wrt_Body_T_vec[1], Lidar_wrt_Body_T_vec[2];
    Lidar_wrt_Body_R << Lidar_wrt_Body_R_vec[0], Lidar_wrt_Body_R_vec[1], Lidar_wrt_Body_R_vec[2],
                        Lidar_wrt_Body_R_vec[3], Lidar_wrt_Body_R_vec[4], Lidar_wrt_Body_R_vec[5],
                        Lidar_wrt_Body_R_vec[6], Lidar_wrt_Body_R_vec[7], Lidar_wrt_Body_R_vec[8];
    
    p_pre_->Lidar_wrt_Body_RT(Lidar_wrt_Body_T, Lidar_wrt_Body_R);

    nh.param<double>("wxx/initial_align/voxelgrid_filter_size", param_voxelgrid_filter_size_, 0.5);
    nh.param<int>("wxx/initial_align/max_iteration", param_max_iterations_, 4);
    nh.param<int>("wxx/initial_align/icp_mode", param_function_value_, 1);
    nh.param<double>("wxx/initial_align/initial_map_size", param_map_size_, 50);
    nh.param<int>("wxx/initial_align/param_accumulated_scan", param_accumulated_scan_, 10);

    std::cout << "\033[1;32m[wxx][initial align] initial_map_pcd_name: " << initial_map_pcd_name_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m[wxx][initial align] max icp iterations: " << param_max_iterations_ << "\033[0m" << std::endl;
    switch(param_function_value_)
    {
        case 1:{
            std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- pcl_gicp ---" << "\033[0m" << std::endl;
            break;
        }
        case 2:{
            std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- pcl_ndt ---" << "\033[0m" << std::endl;
            break;
        }
        case 3:{
            std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- fgicp_st ---" << "\033[0m" << std::endl;
            break;
        }
        case 4:{
            std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- fgicp_mt ---" << "\033[0m" << std::endl;
            break;
            }
        case 5:{
            std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- vgicp_st ---" << "\033[0m" << std::endl;
            break;
        }
    
        default:{
            std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- invalid function ---" << "\033[0m" << std::endl;
            ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
            ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
            ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
            exit(0);
            break;
        }
    }
    std::cout << "\033[1;32m[wxx][initial align] Init_Body_pos: " << Init_Body_pos_.transpose() << "\033[0m" << std::endl;

    pcl::PCDReader reader;
    // string file_name_map_kdtree = initial_map_pcd_name_;
    PointCloudXYZI::Ptr cloud_map(new PointCloudXYZI());
    string all_points_dir_map_kdtree(string(string(ROOT_DIR) + "PCD/") + initial_map_pcd_name_);
    if( reader.read(all_points_dir_map_kdtree, *cloud_map) == -1 )
    {
        std::cout << "\033[1;31m[wxx][initial align] The initial pcd file [" << initial_map_pcd_name_ << "] does not exist!!!" << "\033[0m" << std::endl;
        std::cout << "\033[1;31m[wxx][initial align] Align Initialization Fail!!!" << "\033[0m" << std::endl;

        ROS_ERROR("[wxx][initial align] initial pcd file  does not exist!!!");
        ROS_ERROR("[wxx][initial align] initial pcd file  does not exist!!!");
        ROS_ERROR("[wxx][initial align] initial pcd file  does not exist!!!");
        exit(0);
    }
    for (const auto& pt : cloud_map->points) {
        pcl::PointXYZ xyz_point;
        Eigen::Vector3d tp;
        xyz_point.x = pt.x;
        xyz_point.y = pt.y;
        xyz_point.z = pt.z;
        PCL_to_Eigen(xyz_point, tp);
        if( tp.norm() < param_map_size_ )
            target_cloud_->points.push_back(xyz_point);
    }

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre_->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubOdomAftInit = nh.advertise<nav_msgs::Odometry> 
        ("/initial_odom_for_lio", 100000);
    // 两种输出用来debug，确认从icp里拿出来的odom是正确的
    ros::Publisher pubLaserCloudOdom = nh.advertise<sensor_msgs::PointCloud2>
            ("/initial_cloud_from_odom", 100000);
    ros::Publisher pubLaserCloudICP  = nh.advertise<sensor_msgs::PointCloud2>
            ("/initial_cloud_from_icp", 100000);

//------------------------------------------------------------------------------------------------------
    
    
    ros::Rate rate(2000);
    bool status = ros::ok();
    
    while (status)
    {
        ros::spinOnce();

        if(receive_source_ && imu_init_ready_)
        {
            // downsampling
            pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
            voxelgrid.setLeafSize(param_voxelgrid_filter_size_, param_voxelgrid_filter_size_, param_voxelgrid_filter_size_);

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
            voxelgrid.setInputCloud(target_cloud_);
            voxelgrid.filter(*filtered);
            target_cloud_ = filtered;

            filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
            voxelgrid.setInputCloud(source_cloud_);
            voxelgrid.filter(*filtered);
            source_cloud_ = filtered;

            *source_cloud_raw_ = *source_cloud_;

            // init pose
            RT_to_M4(Init_Body_pos_, Init_Body_rot_by_align_gravity_, result_pose_);
            translate_cloud(source_cloud_, result_pose_);

            std::cout << "target:" << target_cloud_->size() << "[pts] source:" << source_cloud_->size() << "[pts]" << std::endl; 
            switch(param_function_value_)
            {
                case 1:{
                    // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- pcl_gicp ---" << "\033[0m" << std::endl;
                    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> pcl_gicp;
                    test(pcl_gicp, target_cloud_, source_cloud_, param_max_iterations_);
                    break;
                }
                case 2:{
                    // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- pcl_ndt ---" << "\033[0m" << std::endl;
                    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
                    pcl_ndt.setResolution(1.0);
                    test(pcl_ndt, target_cloud_, source_cloud_, param_max_iterations_);
                    break;
                }
                case 3:{
                    // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- fgicp_st ---" << "\033[0m" << std::endl;
                    fast_gicp::FastGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ> fgicp_st;
                    test(fgicp_st, target_cloud_, source_cloud_, param_max_iterations_);
                    break;
                }
                case 4:{
                    // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- fgicp_mt ---" << "\033[0m" << std::endl;
                    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> fgicp_mt;
                    test(fgicp_mt, target_cloud_, source_cloud_, param_max_iterations_);
                    break;
                    }
                case 5:{
                    // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- vgicp_st ---" << "\033[0m" << std::endl;
                    fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
                    vgicp.setResolution(1.0);
                    vgicp.setNumThreads(1);
                    test(vgicp, target_cloud_, source_cloud_, param_max_iterations_);
                    break;
                }
            
                default:{
                    // std::cout << "\033[1;32m[wxx][initial align] icp mode: " << "--- invalid function ---" << "\033[0m" << std::endl;
                    ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
                    ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
                    ROS_ERROR("[wxx][initial align] icp mode INVALID !!!");
                    exit(0);
                    break;
                }
            }   
            break;
        }
    }

    // pub result
    nav_msgs::Odometry odomAftInit;
    Eigen::Quaterniond quaternion(result_pose_.block<3,3>(0,0).cast<double>());
    odomAftInit.pose.pose.orientation.x = quaternion.x();
    odomAftInit.pose.pose.orientation.y = quaternion.y();
    odomAftInit.pose.pose.orientation.z = quaternion.z();
    odomAftInit.pose.pose.orientation.w = quaternion.w();
    odomAftInit.pose.pose.position.x = result_pose_.block<3,1>(0,3)(0);
    odomAftInit.pose.pose.position.y = result_pose_.block<3,1>(0,3)(1);
    odomAftInit.pose.pose.position.z = result_pose_.block<3,1>(0,3)(2);
    Eigen::Vector3d result_pos = result_pose_.block<3,1>(0,3).cast<double>();
    Eigen::Matrix3d result_rot = result_pose_.block<3,3>(0,0).cast<double>();
    ros::Rate rate_pub_odom(10);
    for(int i=0;i<20;i++)
    {
        pubOdomAftInit.publish(odomAftInit);
        publish_frame_world(pubLaserCloudICP);
        publish_frame_world(pubLaserCloudOdom, result_pos, result_rot);

        rate_pub_odom.sleep();
    }

    return 0;
}
