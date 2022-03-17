/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-10-16 23:06:23
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-01-06 11:54:05
 */
#ifndef FUNC_H
#define FUNC_H

#include "location_msgs/RTK.h"
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <map>
#include <unistd.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

using namespace std;

void pose_pub(ros::NodeHandle nh, const vector<Eigen::Quaternion<double>> &q, const vector<Eigen::Vector3d> &t);
pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);
void pointcloud_in_range(const double min_range, const double max_range,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_in,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_out);
void save_sc_pcd_pose(const string &folder_path,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud,
                      const Eigen::Quaternion<double> &q, const Eigen::Vector3d &t);

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ");

std::string padZeros(int val, int num_digits = 6); 

class rtk_pose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    rtk_pose();
    void rtk_data_in(const sensor_msgs::Imu rtk);
    void ll2file(string path);
    bool get_pose(double time, Eigen::Quaternion<double> &q, Eigen::Vector3d &t);

    vector<Eigen::Quaterniond> map_q_;
    vector<Eigen::Vector3d> map_t_;
    vector<double> map_time_;

    vector<double> longitude_;
    vector<double> latitude_;

private:
    bool init_;
    Eigen::Matrix3d R_ENU2IMU_ORIGIN;
    Eigen::Vector3d t_ECEF2ENU_;

    double e_;
    double R_;
    double torad_;
    double Re_;
};

#endif