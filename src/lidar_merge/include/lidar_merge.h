#ifndef LIDAR_MERGE_H
#define LIDAR_MERGE_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rosbag/bag.h>

#include <iostream>
#include <cmath>
#include <limits>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h> //·Öžî
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZI>());

Matrix4d front_left;
Matrix4d front_right;

ros::Publisher pub;
ros::Publisher imu_pub;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = imu_msg->header.frame_id;
    imu_data.linear_acceleration.x = imu_msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg->linear_acceleration.z;

    imu_data.angular_velocity.x = imu_msg->angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg->angular_velocity.z;

    imu_data.orientation_covariance[0] = imu_msg->orientation_covariance[0];
    imu_data.orientation_covariance[1] = imu_msg->orientation_covariance[1];
	imu_data.orientation_covariance[2] = imu_msg->orientation_covariance[2];

    imu_data.orientation.x = imu_msg->orientation.x;
    imu_data.orientation.y = imu_msg->orientation.y;
    imu_data.orientation.z = imu_msg->orientation.z;
    imu_data.orientation.w = imu_msg->orientation.w;

    

    imu_pub.publish(imu_data);

}

void right_callback(const sensor_msgs::PointCloud2ConstPtr &right_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*right_msg,*cloud);
	pcl::transformPointCloud (*cloud,*cloud, front_right);
	right_cloud->points.clear();
	for (auto it = cloud->points.begin(); it < cloud->points.end();it++)
        {
			right_cloud->points.push_back(*it);
	}
}

void left_callback(const sensor_msgs::PointCloud2ConstPtr &left_msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*left_msg,*cloud);
	pcl::transformPointCloud(*cloud,*cloud,front_left);
	left_cloud->points.clear();


	for (auto it = cloud->points.begin(); it < cloud->points.end();it++)
        {
        	left_cloud->points.push_back(*it);
	}


	for (auto it = right_cloud->points.begin(); it < right_cloud->points.end();it++)
        {
			left_cloud->points.push_back(*it);
	}	
	left_cloud->width=left_cloud->points.size();
	left_cloud->height=1;
	sensor_msgs::PointCloud2 output;
	string frame_id="rslidar";
	pcl::toROSMsg (*left_cloud, output);
	output.header.frame_id = frame_id;
    output.header.stamp = ros::Time::now();
	pub.publish (output);
	
}


#endif
