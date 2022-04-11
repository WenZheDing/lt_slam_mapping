/*
 * @Descripttion: 
 * @version: 
 * @Author: WenZhe Ding
 * @Date: 2022-03-31 10:16:00
 */

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lt_visualize");
    ros::NodeHandle nh;
    std::string central_bag_path;
    std::string query_bag_path;
    std::string imu_topic;
    std::string lidar_topic;

    // sensor_msgs::PointCloud2 TargetCloud;
    // sensor_msgs::PointCloud2 TargetCloud2;

    nh.getParam("central_bag_path", central_bag_path);
    nh.getParam("query_bag_path", query_bag_path);
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("lidar_topic", lidar_topic);

    rosbag::Bag bag;
    bag.open(central_bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(imu_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it = view.begin();
    // 按绝对的时间顺序读取
    for (; it != view.end(); ++it)
    {
        auto m = *it;
        std::string topic = m.getTopic();
        if (topic == std::string(imu_topic))
        {
            sensor_msgs::Imu::ConstPtr pRtk = m.instantiate<sensor_msgs::Imu>();
            if (pRtk != NULL)
            {
                calculate_init_pose(*pRtk, false);
                break;
            }
        }
    }

    rosbag::Bag bag2;
    bag2.open(query_bag_path, rosbag::bagmode::Read);
    rosbag::View view2(bag2, rosbag::TopicQuery(topics));
    rosbag::View::iterator it2 = view2.begin();
    for (; it2 != view2.end(); ++it2)
    {
        auto m = *it2;
        std::string topic = m.getTopic();
        if (topic == std::string(imu_topic))
        {
            sensor_msgs::Imu::ConstPtr pRtk = m.instantiate<sensor_msgs::Imu>();
            if (pRtk != NULL)
            {
                calculate_init_pose(*pRtk, true);
                break;
            }
        }
    }
    std::cout << "q:   " << q.x() << "  " << q.y() << "  " << q.z() << "  " << q.w() << "  " << std::endl;
    std::cout << "t:   " << t(0) << "  " << t(1) << "  " << t(2) << "  " << std::endl;
    gtsam::Rot3 testrot = gtsam::Rot3(q);
    std::cout << "euler angle:   " << testrot.roll() << "  " << testrot.pitch() << "  " << testrot.yaw() << std::endl;
    ros::Rate loop_rate(1);

    ros::Publisher filteredcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filteredcloud", 2);

    while (ros::ok())
    {

        // empty_polygon.header.stamp = ros::Time::now();
        // filteredcloud_pub.publish(TargetCloud2);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
