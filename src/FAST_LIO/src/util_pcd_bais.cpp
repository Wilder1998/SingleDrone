#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

using std::string;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "util_pcd_bais");
    ros::NodeHandle nh;

    string initial_map_pcd_name_;
    bool map_bias_en;
    double map_x_bias, map_y_bias, map_z_bias, map_yaw_bias;
    nh.param<string>("wxx/map_bias/initial_map_name", initial_map_pcd_name_, "initial_map.pcd");
    nh.param<double>("wxx/map_bias/map_x_bias", map_x_bias, 0.0);
    nh.param<double>("wxx/map_bias/map_y_bias", map_y_bias, 0.0);
    nh.param<double>("wxx/map_bias/map_z_bias", map_z_bias, 0.0);
    nh.param<double>("wxx/map_bias/map_yaw_bias", map_yaw_bias, 0.0);
    ros::Publisher pub_initial_map = nh.advertise<sensor_msgs::PointCloud2>("initial_map", 1);
    ros::Publisher pub_bias_map    = nh.advertise<sensor_msgs::PointCloud2>("bias_map", 1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr msg_initial_map(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr msg_bias_map(new sensor_msgs::PointCloud2);


    pcl::PCDReader reader;
    std::cout << "read map from /PCD/" << initial_map_pcd_name_ << std::endl;
    string all_points_dir_map_kdtree(string(string(ROOT_DIR) + "PCD/") + initial_map_pcd_name_);
    if( reader.read(all_points_dir_map_kdtree, *cloud) == -1 )
    {
        std::cout << "\033[1;31m[wxx] The initial pcd file [" << initial_map_pcd_name_ << "] does not exist!!!" << "\033[0m" << std::endl;
        exit(0);
    }
    pcl::toROSMsg(*cloud, *msg_initial_map);
    msg_initial_map->header.frame_id = "world";  


    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << cos(map_yaw_bias), -sin(map_yaw_bias), 0,
                    sin(map_yaw_bias),  cos(map_yaw_bias), 0,
                    0,         0,        1;
    for (auto& point : cloud->points) {
        
        Eigen::Vector3d rotatedPoint = rotationMatrix * point.getVector3fMap().cast<double>();
        point.x = rotatedPoint[0]+map_x_bias;
        point.y = rotatedPoint[1]+map_y_bias;
        point.z = rotatedPoint[2]+map_z_bias;
    }
    string file_name = string("bias.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    std::cout << "bias map saved to /PCD/" << file_name << std::endl;
    pcd_writer.writeBinary(all_points_dir, *cloud);

    pcl::toROSMsg(*cloud, *msg_bias_map);
    msg_bias_map->header.frame_id = "world";  

    ros::Rate rate(1); 
    while (ros::ok())
    {
        // msg->header.stamp = ros::Time::now();

        pub_initial_map.publish(msg_initial_map);
        pub_bias_map.publish(msg_bias_map);

        rate.sleep();
    }

    return 0;
}