#ifndef CALI_TWO_H
#define CALI_TWO_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <math.h>
#include <algorithm>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h> //�ָ�
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr rs_cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> rs_viewer(new pcl::visualization::PCLVisualizer("rs_viewer"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> velo_viewer(new pcl::visualization::PCLVisualizer("velo_viewer"));

pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_velo(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_rs(new pcl::PointCloud<pcl::PointXYZ>);

void rs_Callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*input,*cloud);
	for (auto it = cloud->points.begin(); it < cloud->points.end();it++)
        {
		rs_cloud->points.push_back(*it);
	}
	rs_cloud->width=rs_cloud->points.size();
	rs_cloud->height=1;
}
void velo_Callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	// pcl::fromROSMsg(*input,*velo_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*input,*cloud);
	for (auto it = cloud->points.begin(); it < cloud->points.end();it++)
        {
		velo_cloud->points.push_back(*it);
	}
	velo_cloud->width=velo_cloud->points.size();
	velo_cloud->height=1;
}

void pp_rs(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;
	if (event.getPointsIndices(indices)==-1)
		return;

    for (int i = 0; i < indices.size(); ++i)
    {
        clicked_points_rs->points.push_back(rs_cloud->points.at(indices[i]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_rs, 255, 0, 0);
    std::string cloudName;
    cloudName = "rs_selected";

    rs_viewer->addPointCloud(clicked_points_rs, red, cloudName);
    rs_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
}

void pp_velo(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;
	if (event.getPointsIndices(indices)==-1)
		return;

    for (int i = 0; i < indices.size(); ++i)
    {
        clicked_points_velo->points.push_back(velo_cloud->points.at(indices[i]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_velo, 255, 0, 0);
    std::string cloudName;
    cloudName = "velo_selected";

    velo_viewer->addPointCloud(clicked_points_velo, red, cloudName);
    velo_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
}

double vectorAngle(Vector3d v1, Vector3d v2)
{
    double radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
    if (v1.cross(v2).z() < 0) {
        radian_angle = 2 * M_PI - radian_angle;
    }
    return radian_angle;
}

#endif
