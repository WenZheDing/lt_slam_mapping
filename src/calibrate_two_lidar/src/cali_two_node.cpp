#include "cali_two.h"

#include <chrono>
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pctransform");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1);
	string frame_id="rslidar";

	ros::Subscriber velo_sub = nh.subscribe("/front_right/rslidar_points", 1, velo_Callback);///left/velodyne_points
	ros::Subscriber rs_sub = nh.subscribe("/front_left/rslidar_points", 1, rs_Callback);///rslidar_points

	while(rs_cloud->points.size()==0)
	{
		ros::spinOnce();
		cout<<"no left pointcloud"<<endl;
		loop_rate.sleep();
	}

	while(velo_cloud->points.size()==0)
	{
		ros::spinOnce();
		cout<<"no right pointcloud"<<endl;
		loop_rate.sleep();
	}

	//visualizer
	std::cout << "'x'+click" << std::endl;
	std::cout << "Select points on ground, 'q' to finish" << std::endl;

    velo_viewer->addPointCloud(velo_cloud, "right");
	velo_viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	velo_viewer->registerAreaPickingCallback(pp_velo, (void*)&velo_cloud);
	while (!velo_viewer->wasStopped())
	{
		velo_viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	rs_viewer->addPointCloud(rs_cloud, "left");
	rs_viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	rs_viewer->registerAreaPickingCallback(pp_rs, (void*)&rs_cloud);
	while (!rs_viewer->wasStopped())
	{
		rs_viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	cout<<"clicked rs size:"<<clicked_points_rs->points.size()<<endl;
	cout<<"clicked velo size:"<<clicked_points_velo->points.size()<<endl;

	//ransac拟合
	Eigen::Vector3d rsGroundVector;
	pcl::ModelCoefficients::Ptr rs_ground (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations(300);
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud (clicked_points_rs);
	seg.segment (*plane_inliers, *rs_ground);
	int sign = rs_ground->values[2]/abs(rs_ground->values[2]);
	rsGroundVector(0)=rs_ground->values[0]*sign;
	rsGroundVector(1)=rs_ground->values[1]*sign;
	rsGroundVector(2)=rs_ground->values[2]*sign;
	rsGroundVector.normalize();
	cout<<"rs_ground coefficients:  "<<rs_ground->values[0]<<"\t"<<rs_ground->values[1]<<"\t"<<rs_ground->values[2]<<"\t"<<rs_ground->values[3]<<"\t"<<endl;

	Eigen::Vector3d veloGroundVector;
	pcl::ModelCoefficients::Ptr velo_ground (new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations(300);
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud (clicked_points_velo);
	seg.segment (*plane_inliers, *velo_ground);
	sign = velo_ground->values[2]/abs(velo_ground->values[2]);
	veloGroundVector(0)=velo_ground->values[0]*sign;
	veloGroundVector(1)=velo_ground->values[1]*sign;
	veloGroundVector(2)=velo_ground->values[2]*sign;
	veloGroundVector.normalize();
	cout<<"y plane coefficients:  "<<velo_ground->values[0]<<"\t"<<velo_ground->values[1]<<"\t"<<velo_ground->values[2]<<"\t"<<velo_ground->values[3]<<"\t"<<endl;
	cout<<"rsGroundVector: "<<rsGroundVector.matrix()<<endl;
	cout<<"veloGroundVector: "<<veloGroundVector.matrix()<<endl;


	//计算旋转
	Vector3d xy(0,0,1);
	Vector3d rotationAxis_rs=rsGroundVector.cross(xy);
	double angle_rs=acos(xy.dot(rsGroundVector));
	cout<<"angle rs:  "<<angle_rs<<endl;
	AngleAxisd rotationVector_rs(angle_rs, rotationAxis_rs.normalized());
	Vector3d rotationAxis_velo=veloGroundVector.cross(xy);
	double angle_velo=acos(xy.dot(veloGroundVector));
	cout<<"angle velo:  "<<angle_velo<<endl;
	AngleAxisd rotationVector_velo(angle_velo, rotationAxis_velo.normalized());


	Eigen::Affine3d transform_rs = Eigen::Affine3d::Identity();
	transform_rs.rotate(rotationVector_rs);
	cout<<"rs transform:  "<<transform_rs.matrix()<<endl;

    Eigen::Affine3d transform_velo = Eigen::Affine3d::Identity();
	transform_velo.rotate(rotationVector_velo);
	transform_velo.translation()<<0,0,abs(velo_ground->values[3])-abs(rs_ground->values[3]);
	cout<<"velo transform:  "<<transform_velo.matrix()<<endl;


	//发布
	pcl::PointCloud<pcl::PointXYZ>::Ptr rs_transformed(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr velo_transformed(new pcl::PointCloud<pcl::PointXYZ>());
	ros::Publisher pub_rs = nh.advertise<sensor_msgs::PointCloud2> ("rs_transformed", 1);
	ros::Publisher pub_velo = nh.advertise<sensor_msgs::PointCloud2> ("velo_transformed", 1);

	while(ros::ok())
	{
		pcl::transformPointCloud (*rs_cloud,*rs_transformed, transform_rs);
		pcl::transformPointCloud (*velo_cloud,*velo_transformed, transform_velo);

		sensor_msgs::PointCloud2 output1;
		output1.header.frame_id = frame_id;
		pcl::toROSMsg (*rs_transformed, output1);
		pub_rs.publish (output1);

		sensor_msgs::PointCloud2 output2;	
		pcl::toROSMsg (*velo_transformed, output2);
		output2.header.frame_id = frame_id;
		pub_velo.publish (output2);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

