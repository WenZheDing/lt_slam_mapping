/*
 * @Descripttion: 
 * @version: 
 * @Author: WenZhe Ding
 * @Date: 2022-03-31 10:16:00
 */

#include "lt_visualize.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lt_visualize");
    ros::NodeHandle nh;
    std::string central_bag_path;
    std::string query_bag_path;
    std::string imu_topic;
    std::string lidar_topic;
    std::string central_pcd_path;
    std::string query_pcd_path;
    std::string query_init_pcd_path;

    sensor_msgs::PointCloud2 CentralCloud;
    sensor_msgs::PointCloud2 QueryCloud;
    sensor_msgs::PointCloud2 QueryInitCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr central_pcd(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr query_pcd(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr query_init_pcd(new pcl::PointCloud<pcl::PointXYZI>);

    ros::Publisher central_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/central_pcd", 2);
    ros::Publisher query_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/query_pcd", 2);
    ros::Publisher query_init_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/query_init_pcd", 2);

    nh.getParam("central_bag_path", central_bag_path);
    nh.getParam("query_bag_path", query_bag_path);
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("central_pcd_path", central_pcd_path);
    nh.getParam("query_pcd_path", query_pcd_path);
    nh.getParam("query_init_pcd_path", query_init_pcd_path);

    central_pcd.reset(new pcl::PointCloud<pcl::PointXYZI>());
    query_pcd.reset(new pcl::PointCloud<pcl::PointXYZI>());
    query_init_pcd.reset(new pcl::PointCloud<pcl::PointXYZI>());

    ros::Rate loop_rate(0.1);

    rosbag::Bag bag;
    bag.open(central_bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(imu_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it = view.begin();
    double x1,y1,z1;
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
                transform_llh_to_xyz(pRtk->orientation.y,pRtk->orientation.x,pRtk->orientation.z,x1,y1,z1);
                break;
            }
        }
    }

    rosbag::Bag bag2;
    bag2.open(query_bag_path, rosbag::bagmode::Read);
    rosbag::View view2(bag2, rosbag::TopicQuery(topics));
    rosbag::View::iterator it2 = view2.begin();
    double x2,y2,z2;
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
                transform_llh_to_xyz(pRtk->orientation.y,pRtk->orientation.x,pRtk->orientation.z,x2,y2,z2);
                break;
            }
        }
    }
    // 由rtk算得的第一帧相对q和t
    std::cout << "q:   " << q.x() << "  " << q.y() << "  " << q.z() << "  " << q.w() << "  " << std::endl;
    std::cout << "t:   " << t(0) << "  " << t(1) << "  " << t(2) << "  " << std::endl;
    gtsam::Rot3 testrot = gtsam::Rot3(q);
    std::cout << "euler angle:   " << testrot.roll() << "  " << testrot.pitch() << "  " << testrot.yaw() << std::endl;

    //publish pointcloud
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(central_pcd_path, *central_pcd) == -1)
    {
        PCL_ERROR("Could not read file :  %s \n", central_pcd_path.c_str());
        return 0;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(query_pcd_path, *query_pcd) == -1)
    {
        PCL_ERROR("Could not read file :  %s \n", query_pcd_path.c_str());
        return 0;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(query_init_pcd_path, *query_init_pcd) == -1)
    {
        PCL_ERROR("Could not read file :  %s \n", query_init_pcd_path.c_str());
        return 0;
    }

    transform_origin(central_pcd,x1,y1,z1);
    transform_origin(query_pcd,x1,y1,z1);
    transform_origin(query_init_pcd,x2,y2,z2);
    
    pcl::toROSMsg(*central_pcd, CentralCloud);
    CentralCloud.header.stamp = ros::Time::now();
    CentralCloud.header.frame_id = "base";

    pcl::toROSMsg(*query_pcd, QueryCloud);
    QueryCloud.header.stamp = ros::Time::now();
    QueryCloud.header.frame_id = "base";

    pcl::toROSMsg(*query_init_pcd, QueryInitCloud);
    QueryInitCloud.header.stamp = ros::Time::now();
    QueryInitCloud.header.frame_id = "base";


    while (ros::ok())
    {
        CentralCloud.header.stamp = ros::Time::now();
        QueryCloud.header.stamp = ros::Time::now();
        QueryInitCloud.header.stamp = ros::Time::now();
        central_pcd_pub.publish(CentralCloud);
        query_pcd_pub.publish(QueryCloud);
        query_init_pcd_pub.publish(QueryInitCloud);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
