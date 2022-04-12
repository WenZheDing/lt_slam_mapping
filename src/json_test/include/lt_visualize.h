#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PolygonStamped.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <thread>

#include <pcl/point_types.h>
#include <json/json.h>
#include <fstream>
#include <iterator>
#include <iomanip>

Eigen::Matrix3d R_ENU2IMU_ORIGIN;
Eigen::Vector3d t_ECEF2ENU_;

#define GLOBAL_ORIGIN_LAT 31.1367846
#define GLOBAL_ORIGIN_LON 118.1789369

double e_ = 0.0818191908425;
double R_ = 6378137;
double torad_ = M_PI / 180;
double Re_ = 0;
Eigen::Vector3d t(0, 0, 0);
Eigen::Quaternion<double> q(1, 0, 0, 0);

void calculate_init_pose(const sensor_msgs::Imu rtk, bool init_flag)
{
    if (!init_flag)
    {
        //东北天相对于第一帧组合导航的旋转矩阵
        R_ENU2IMU_ORIGIN = Eigen::AngleAxisd(rtk.orientation.w * torad_, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(rtk.orientation_covariance[1] * torad_, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(rtk.orientation_covariance[0] * torad_, Eigen::Vector3d::UnitX());
        Re_ = R_ / sqrt(1 - e_ * e_ * sin(rtk.orientation.x * torad_) * sin(rtk.orientation.x * torad_));
        t_ECEF2ENU_[0] = (Re_ + rtk.orientation.z) * cos(rtk.orientation.x * torad_) * cos(rtk.orientation.y * torad_);
        t_ECEF2ENU_[1] = (Re_ + rtk.orientation.z) * cos(rtk.orientation.x * torad_) * sin(rtk.orientation.y * torad_);
        t_ECEF2ENU_[2] = (Re_ * (1 - e_ * e_) + rtk.orientation.z) * sin(rtk.orientation.x * torad_);
    }
    else
    {
        Eigen::Matrix3d R_ENU;
        Eigen::Matrix3d R_IMU;

        R_ENU = Eigen::AngleAxisd(rtk.orientation.w * torad_, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rtk.orientation_covariance[1] * torad_, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rtk.orientation_covariance[0] * torad_, Eigen::Vector3d::UnitX());
        R_IMU = R_ENU * R_ENU2IMU_ORIGIN.inverse();

        q = R_IMU.inverse();

        Eigen::Vector3d t_ECEF(0, 0, 0);
        Eigen::Vector3d t_ENU(0, 0, 0);

        t_ECEF[0] = (Re_ + rtk.orientation.z) * cos(rtk.orientation.x * torad_) * cos(rtk.orientation.y * torad_) - t_ECEF2ENU_[0];
        t_ECEF[1] = (Re_ + rtk.orientation.z) * cos(rtk.orientation.x * torad_) * sin(rtk.orientation.y * torad_) - t_ECEF2ENU_[1];
        t_ECEF[2] = (Re_ * (1 - e_ * e_) + rtk.orientation.z) * sin(rtk.orientation.x * torad_) - t_ECEF2ENU_[2];

        t_ENU[0] = -sin(rtk.orientation.y * torad_) * t_ECEF[0] + cos(rtk.orientation.y * torad_) * t_ECEF[1];
        t_ENU[1] = -sin(rtk.orientation.x * torad_) * cos(rtk.orientation.y * torad_) * t_ECEF[0] - sin(rtk.orientation.x * torad_) * sin(rtk.orientation.y * torad_) * t_ECEF[1] + cos(rtk.orientation.x * torad_) * t_ECEF[2];
        t_ENU[2] = cos(rtk.orientation.x * torad_) * cos(rtk.orientation.y * torad_) * t_ECEF[0] + cos(rtk.orientation.x * torad_) * sin(rtk.orientation.y * torad_) * t_ECEF[1] + sin(rtk.orientation.x * torad_) * t_ECEF[2];

        t = R_ENU2IMU_ORIGIN * t_ENU;
    }
}

void transform_origin(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,double &origin_x_in_world, double origin_y_in_world, double &origin_z_in_world)
{
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
    // cloud_out->resize(cloud_in->points.size());

    for (uint32_t i = 0; i < cloud_in->points.size(); i++)
    {
        Eigen::Vector3d point(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);

        pcl::PointXYZI p;
        p.x = point[0] + origin_x_in_world;
        p.y = point[1] + origin_y_in_world;
        p.z = point[2] + origin_z_in_world;
        p.intensity = cloud_in->points[i].intensity;

        cloud_in->points[i] = p;
    }

    // cloud_out->height = 1;
    // cloud_out->width = cloud_in->points.size();
    // return cloud_out;
}

void transform_llh_to_xyz(const double longitude, const double latitude, const double altitude, double &origin_x, double &origin_y, double &origin_z)
{
    float R0 = 6378137.0;
    float e = 0.0818191908425;

    float L0 = GLOBAL_ORIGIN_LAT;
    float lamda0 = GLOBAL_ORIGIN_LON;
    float hb = 0.0;
    float RE0 = R0 / (sqrt(1.0 - e * e * sin(L0 * M_PI / 180) * sin(L0 * M_PI / 180.0)));
    float x0 = (RE0 + hb) * cos(L0 * M_PI / 180.0) * cos(lamda0 * M_PI / 180.0);
    float y0 = (RE0 + hb) * cos(L0 * M_PI / 180.0) * sin(lamda0 * M_PI / 180.0);
    float z0 = ((1.0 - e * e) * RE0 + hb) * sin(L0 * M_PI / 180.0);

    float L = latitude;
    float lamda = longitude;
    float h = altitude;
    float RE = R0 / (sqrt(1.0 - e * e * sin(L * M_PI / 180.0) * sin(L * M_PI / 180.0)));
    float x = (RE + h) * cos(L * M_PI / 180.0) * cos(lamda * M_PI / 180.0);
    float y = (RE + h) * cos(L * M_PI / 180.0) * sin(lamda * M_PI / 180.0);
    float z = ((1.0 - e * e) * RE + h) * sin(L * M_PI / 180.0);

    float dx = x - x0;
    float dy = y - y0;
    float dz = z - z0;
    float dn = -sin(L * M_PI / 180.0) * cos(lamda * M_PI / 180.0) * dx - sin(L * M_PI / 180.0) * sin(lamda * M_PI / 180.0) * dy + cos(L * M_PI / 180.0) * dz;
    float de = -sin(lamda * M_PI / 180.0) * dx + cos(lamda * M_PI / 180.0) * dy;

    origin_x = de;
    origin_y = dn;
    origin_z = 0.0;

    return;
}