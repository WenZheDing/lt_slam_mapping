/*
 * @Descripttion: 
 * @version: 
 * @Author: Yuhang Li
 * @Date: 2021-12-25 20:31:21
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-01-20 10:12:30
 */
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <json/json.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#define GLOBAL_ORIGIN_LAT 31.1367846
#define GLOBAL_ORIGIN_LON 118.1789369

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "json_test");
  ros::NodeHandle nh;
  double point1_x, point1_y, point1_lat, point1_lon, z;
  bool transform_from_llh;
  std::string json_file_path;
  std::string json_file_path2;

  nh.getParam("point1_x", point1_x);
  nh.getParam("point1_y", point1_y);
  nh.getParam("transform_from_llh", transform_from_llh);
  nh.getParam("point1_lat", point1_lat);
  nh.getParam("point1_lon", point1_lon);
  nh.getParam("json_file_path", json_file_path);
  nh.getParam("json_file_path2", json_file_path2);

  ros::Rate loop_rate(1.0);
  ros::Publisher occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/ogm_json", 1);
  ros::Publisher occupancy_grid_pub2 = nh.advertise<nav_msgs::OccupancyGrid>("/ogm_json2", 1);
  ros::Publisher pubStart = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
  ros::Publisher pubwaji = nh.advertise<geometry_msgs::PointStamped>("/waji", 1);

  nav_msgs::OccupancyGrid global_map;
  nav_msgs::OccupancyGrid global_map2;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PointStamped waji;

  Json::Reader json_reader;
  std::ifstream json_file;
  std::ifstream json_file2;
  Json::Value root;
  Json::Value root2;
  json_file.open(json_file_path);

  if (!json_reader.parse(json_file, root))
  {
    std::cout << "Error opening json file : " << json_file_path << std::endl;
  }

  int width, height, size;
  float resolution;
  global_map.info.width = root["width"].asInt();
  global_map.info.height = root["height"].asInt();
  global_map.info.resolution = root["resolution"].asFloat();
  global_map.header.frame_id = "/map";
  global_map.info.origin.position.x = root["origin.x"].asDouble();
  global_map.info.origin.position.y = root["origin.y"].asDouble();
  global_map.info.origin.position.z = 0.0;
  size = root["data"].size();

  for (int i = 0; i < size; i++)
  {
    global_map.data.push_back(root["data"][i].asInt());
  }

  //open map2
  json_file2.open(json_file_path2);

  if (!json_reader.parse(json_file2, root2))
  {
    std::cout << "Error opening json file : " << json_file_path2 << std::endl;
  }

  global_map2.info.width = root2["width"].asInt();
  global_map2.info.height = root2["height"].asInt();
  global_map2.info.resolution = root2["resolution"].asFloat();
  global_map2.header.frame_id = "/map";
  global_map2.info.origin.position.x = root2["origin.x"].asDouble();
  global_map2.info.origin.position.y = root2["origin.y"].asDouble();
  global_map2.info.origin.position.z = 0.0;
  size = root2["data"].size();

  for (int i = 0; i < size; i++)
  {
    global_map2.data.push_back(root2["data"][i].asInt());
  }

  // Eigen::Quaterniond q_imu = Eigen::AngleAxisd(1.91166, Eigen::Vector3d::UnitZ()) *
  //                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
  //                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  // goal.header.frame_id = "/map";
  // goal.header.stamp = ros::Time::now();
  // goal.pose.position.x = -14.9782;
  // goal.pose.position.y = 112.553;
  // goal.pose.orientation.x = q_imu.x();
  // goal.pose.orientation.y = q_imu.y();
  // goal.pose.orientation.z = q_imu.z();
  // goal.pose.orientation.w = q_imu.w();

  waji.header.frame_id = "/map";
  waji.header.stamp = ros::Time::now();
  if (!transform_from_llh)
  {
    waji.point.x = point1_x;
    waji.point.y = point1_y;
  }
  else
  {
    transform_llh_to_xyz(point1_lon, point1_lat, 0, waji.point.x, waji.point.y, z);
    std::cout << "x= " << waji.point.x <<"    "<< "y= " << waji.point.y << std::endl;
  }

  while (ros::ok())
  {
    occupancy_grid_pub.publish(global_map);
    occupancy_grid_pub2.publish(global_map2);
    // pubStart.publish(goal);
    pubwaji.publish(waji);

    loop_rate.sleep();

    ros::spinOnce();
  }

  return 0;
}
