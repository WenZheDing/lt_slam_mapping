#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include <ros/ros.h>  
#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <sensor_msgs/PointCloud2.h>  
#include <pcl/io/pcd_io.h>

using namespace std;

int main(int argc, char **argv){
    string topic, path, frame_id;
    
    ros::init(argc,argv,"publish_pointcloud");
    ros::NodeHandle nh;
    cout<<"debug1"<<endl;
    nh.param<string>("path",path,"/home/liyuhang/erasor_ws/05_result.pcd");
    nh.param<string>("frame_id",frame_id,"/map");
    nh.param<string>("topic",topic,"/pointcloud/output");
    cout<<"debug2"<<endl;

    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>(topic,10);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cout<<"debug3"<<endl;

    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile (path, cloud);
    cout<<"debug4"<<endl;

    pcl::toROSMsg(cloud,output);

    output.header.stamp = ros::Time::now();
    output.header.frame_id = frame_id;

    cout<<"path = "<<path<<endl;
	cout<<"frame_id = "<<frame_id<<endl;
	cout<<"topic = "<<topic<<endl;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {   
        pcd_pub.publish(output);
        ros::spinOnce;
        loop_rate.sleep();
    }
    
    return 0;


}