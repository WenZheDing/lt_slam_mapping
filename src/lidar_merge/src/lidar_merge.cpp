/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2021-10-16 23:06:23
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-01-20 15:24:46
 */
#include<lidar_merge.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "lidar_merge");
	ros::NodeHandle nh;
	cout<<"lidar_merge node started!"<<endl;
	//ros::Rate loop_rate(20);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/all_lidars", 1);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data_processed",100);

	ros::Subscriber right_sub = nh.subscribe("/os_cloud_node2/points", 1, right_callback);
	ros::Subscriber left_sub = nh.subscribe("/os_cloud_node3/points", 1, left_callback);
    ros::Subscriber IMU_sub = nh.subscribe("/imu_data",100,imu_callback);

	
	front_left << 	0.663426  , -0.748219 , 0.00581334 ,   -2.54632,
  					0.748219  , 0.663325  , -0.012919  ,  -0.135617,
					0.00581009, 0.0129204 ,     0.9999 ,  0.0642934,
         			0         , 0         ,      0     ,          1;
	
	front_right <<  0.637424  ,  0.770513  ,  0     ,    2.5,
				   -0.770513  ,  0.637424  ,  0     ,     -0,
        			0     	  ,  0         ,  1     ,     -0,
        			0         ,  0         ,  0     ,      1;
	

	ros::spin();
	return 0;

}