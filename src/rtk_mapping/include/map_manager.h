/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-10-16 23:06:23
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-01-22 15:51:47
 */
#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <vector>
#include <map>
#include <set>
#include <string>
#include <cmath>

#include <stdio.h>

#include <thread>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <ndt_cpu/NormalDistributionsTransform.h>

#include <eigen3/Eigen/Dense>

using std::map;
using std::set;
using std::string;
using std::vector;

class map_manager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    map_manager(string pcd_path, string submap_path, string sc_path);

    void load_map();
    void generate_submap();
    void generate_matchcloud(const double &pose_x, const double &pose_y, bool &isNew, pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud);
    void init_location(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                                Eigen::Quaterniond &q_out, Eigen::Vector3d &t_out);
    //TODO delete
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_;

private:
    string pcd_path_;
    string submap_path_;
    string sc_path_;

    map<string, pcl::PointCloud<pcl::PointXYZI>::Ptr> map_name_cloud_;
    map<string, pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator it_;
    set<string> submap_name_;

    int map_centerx_;
    int map_centery_;

    vector<Eigen::Quaternion<double> > sc_q_;
    vector<Eigen::Vector3d> sc_t_;
};

#endif