/*
 * @Descripttion: 
 * @version: 
 * @Author: WenZhe Ding
 * @Date: 2022-03-22 15:37:00
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PolygonStamped.h>

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
#include "common_data.h"

#define GLOBAL_ORIGIN_LAT 31.1367846
#define GLOBAL_ORIGIN_LON 118.1789369
#define foreach BOOST_FOREACH

// #define M_PI 3.14159265358979
#define MAX_OGM 10000
#define MIN_OGM -10000
#define TO_RAD M_PI / 180

nav_msgs::OccupancyGrid o_global_map_;
nav_msgs::OccupancyGrid o_local_map_;
MapManagerDataType::Imu o_imu_msg_;
float global_map_heading;
pcl::PointCloud<pcl::PointXYZI>::Ptr pLaserCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud;
geometry_msgs::PointStamped local_origin;

// ogm 参数
double d_width1, d_width2;
double d_height1, d_height2;
double d_z1, d_z2;
double d_ego_left, d_ego_right, d_ego_front, d_ego_back, d_ego_top, d_ego_bottom;
double d_ego_empty_left, d_ego_empty_right, d_ego_empty_front, d_ego_empty_back;
double d_park_height;
double d_resolution;
double d_height_diff;
// mapping中的障碍物检测参数;
int i_max_thresh;
int i_mid_thresh;
int i_min_thresh;
int i_classify_park_thresh;

void transform_llh_to_xyz(const double longitude, const double latitude, const double altitude, float &origin_x, float &origin_y, float &origin_z)
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
    // std::cout << "longitude:  " << std::setprecision(10) << longitude << "     latitude:  " << std::setprecision(10) << latitude << std::endl;
    // std::cout << "origin_x:  " << origin_x << "     origin_y:  " << origin_y << std::endl;

    return;
}

void pointcloud_in_range(const double min_range, const double max_range,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_in,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_out)
{
    pcloud_out->clear();
    for (uint32_t i = 0; i < pcloud_in->points.size(); i++)
    {
        double x = pcloud_in->points[i].x;
        double y = pcloud_in->points[i].y;
        double s = sqrt(x * x + y * y);
        // std::cout << "x and y is " << x <<"  and  "<< y<< std::endl;
        if (s >= min_range && s <= max_range)
            pcloud_out->points.push_back(pcloud_in->points[i]);
    }

    pcloud_out->height = 1;
    pcloud_out->width = pcloud_out->points.size();
}

void CreateLocalMap()
{
    int rows = (d_height2 - d_height1) / d_resolution;
    int cols = (d_width2 - d_width1) / d_resolution;
    std::vector<std::vector<float>> maxheight(rows);
    std::vector<std::vector<float>> minheight(rows);
    std::vector<std::vector<std::vector<float>>> classify_max(rows), classify_min(rows), classify_mid(rows), classify_park(rows);
    for (int i = 0; i < rows; i++)
    {
        maxheight[i].resize(cols);
        minheight[i].resize(cols);
        classify_max[i].resize(cols);
        classify_min[i].resize(cols);
        classify_mid[i].resize(cols);
        classify_park[i].resize(cols);
        for (int j = 0; j < cols; j++)
        {
            maxheight[i][j] = MIN_OGM;
            minheight[i][j] = MAX_OGM;
        }
    }
    for (int i = 0; i < pLaserCloud->points.size(); i++)
    {
        Eigen::Vector4f p;
        // p << o_lidar_msg_.data.points[i].x, o_lidar_msg_.data.points[i].y, o_lidar_msg_.data.points[i].z, 1.0;
        auto point = pLaserCloud->points[i];
        // auto point = o_transform_matrix_lidar_front_right * p;
        // auto point = g_lidarPointsMsg[i];
        // 清除自车位置
        if (point.x < d_ego_front && point.x > d_ego_back && point.y < d_ego_left &&
            point.y > d_ego_right && point.z < d_ego_top && point.z > d_ego_bottom)
        {
            continue;
        }
        //清除车辆周围和前方的矩形区域内的点云
        if (point.x < d_ego_empty_front && point.x > d_ego_empty_back && point.y < d_ego_empty_left && point.y > d_ego_empty_right)
        {
            continue;
        }
        if (point.x > d_width1 + 1 && point.x < d_width2 - 1 && point.y > d_height1 + 1 &&
            point.y < d_height2 - 1 && point.z > d_z1 && point.z < d_z2)
        {
            int x = (point.x - d_width1) / d_resolution;
            int y = (point.y - d_height1) / d_resolution;
            if (point.z > maxheight[y][x])
            {
                maxheight[y][x] = point.z;
            }
            if (point.z < minheight[y][x])
            {
                minheight[y][x] = point.z;
            }
            // 高于一定高度的都认为障碍物
            if (point.z >= d_park_height)
            {
                classify_park[y][x].push_back(point.z);
            }
            // 对范围内的点进行高度上的分类，若三段高度内都有点，认为是人形或车型障碍物;
            if (point.z >= d_z1 && point.z < (2 * d_z1 + d_z2) / 3.0)
            {
                classify_min[y][x].push_back(point.z);
            }
            else if (point.z >= (2 * d_z1 + d_z2) / 3.0 && point.z < (2 * d_z2 + d_z1) / 3.0)
            {
                classify_mid[y][x].push_back(point.z);
            }
            else if (point.z >= (2 * d_z2 + d_z1) / 3.0 && point.z <= d_z2)
            {
                classify_max[y][x].push_back(point.z);
            }
        }
    }
    o_local_map_.header.frame_id = "base";
    o_local_map_.header.stamp = ros::Time::now();
    o_local_map_.info.resolution = d_resolution;
    o_local_map_.info.width = cols;
    o_local_map_.info.height = rows;
    o_local_map_.info.origin.position.x = d_width1;
    o_local_map_.info.origin.position.y = d_height1;
    o_local_map_.data.resize(rows * cols);
    for (int i = 0; i < rows; i++)
    {
        // std::cout << std::endl;
        for (int j = 0; j < cols; j++)
        {
            int index = j + o_local_map_.info.width * i;
            o_local_map_.data[index] = 0;
            if ((maxheight[i][j] > minheight[i][j]) && (maxheight[i][j] - minheight[i][j] > d_height_diff))
            {
                // map.data[index] = int((maxheight[i][j] - minheight[i][j]) * 100);
                // 不是0，其他值对碰撞检测都一样
                o_local_map_.data[index] = 50;
                // std::cout << "+";
            }
            if (classify_park[i][j].size() >= i_classify_park_thresh)
            {
                o_local_map_.data[index] = 100;
                // std::cout << "park is " << classify_park[i][j].size() << std::endl;
            }
            // else
            // {
            //     if (i == static_cast<int>(-d_width1 / d_resolution) &&
            //         j == static_cast<int>(-d_height1 / d_resolution))
            //         std::cout << "%";
            //     else
            //         std::cout << ".";
            // }
            if (classify_min[i][j].size() >= i_min_thresh && classify_mid[i][j].size() >= i_mid_thresh &&
                classify_max[i][j].size() >= i_max_thresh)
            {
                std::cout << "max is " << classify_max[i][j].size()
                          << "\tmid is " << classify_mid[i][j].size()
                          << "\tmin is " << classify_min[i][j].size()
                          << std::endl;
                printf("The height range of this grid is (%.2f, %.2f)", minheight[i][j], maxheight[i][j]);
                // o_local_map_.data[index] = 100;
            }
        }
    }
    // std::cout << std::endl;
}

void UpdateGloballMap()
{
    int rows = o_local_map_.info.height;
    int cols = o_local_map_.info.width;
    int local_x_in_global, local_y_in_global;
    float delta_x, delta_y;
    float delta_heading = (o_imu_msg_.AngleHeading - global_map_heading) * TO_RAD;
    float global_origin_x = o_global_map_.info.origin.position.x;
    float global_origin_y = o_global_map_.info.origin.position.y;
    float local_origin_x = o_imu_msg_.x - (d_width2)*cos((o_imu_msg_.AngleHeading) * TO_RAD) - (d_height2)*sin((o_imu_msg_.AngleHeading) * TO_RAD);
    float local_origin_y = o_imu_msg_.y + (d_width2)*sin((o_imu_msg_.AngleHeading) * TO_RAD) - (d_height2)*cos((o_imu_msg_.AngleHeading) * TO_RAD);
    local_origin.header.frame_id = "/map";
    local_origin.header.stamp = ros::Time::now();
    local_origin.point.x = local_origin_x;
    local_origin.point.y = local_origin_y;
    delta_x = sqrt(pow(local_origin_x - global_origin_x, 2) + pow(local_origin_y - global_origin_y, 2)) *
              cos(global_map_heading * TO_RAD + atan((local_origin_y - global_origin_y) / (local_origin_x - global_origin_x)));
    delta_y = sqrt(pow(local_origin_x - global_origin_x, 2) + pow(local_origin_y - global_origin_y, 2)) *
              sin(global_map_heading * TO_RAD + atan((local_origin_y - global_origin_y) / (local_origin_x - global_origin_x)));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            local_x_in_global = floor((cos(delta_heading) * j * d_resolution + sin(delta_heading) * i * d_resolution + delta_x) / o_global_map_.info.resolution);
            local_y_in_global = floor((-sin(delta_heading) * j * d_resolution + cos(delta_heading) * i * d_resolution + delta_y) / o_global_map_.info.resolution);
            if (local_x_in_global < 0 || local_y_in_global < 0 || local_x_in_global > o_global_map_.info.width ||
                local_y_in_global > o_global_map_.info.height)
            {
                continue;
            }
            else
            {
                // o_global_map_.data[local_y_in_global * o_global_map_.info.width + local_x_in_global] =
                //     o_global_map_.data[local_y_in_global * o_global_map_.info.width + local_x_in_global] > o_local_map_.data[i * cols + j] ? o_global_map_.data[local_y_in_global * o_global_map_.info.width + local_x_in_global] : o_local_map_.data[i * cols + j];
                // 这里不存在误差放大，因为会通过类似取余的操作来反推出y和x，把取整操作放在下面会导致x不准
                o_global_map_.data[local_y_in_global * o_global_map_.info.width + local_x_in_global] = o_local_map_.data[i * cols + j];
            }
        }
    }
}

void publishTfFcn()
{
    ros::Rate loopRate(100);
    Eigen::AngleAxisf o_rotation_vector((-o_imu_msg_.AngleHeading) / 180 * M_PI, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf o_quat(o_rotation_vector);
    while (ros::ok())
    {
        tf::TransformBroadcaster broadcaster;
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(o_quat.x(), o_quat.y(), o_quat.z(), o_quat.w()), tf::Vector3(o_imu_msg_.x, o_imu_msg_.y, o_imu_msg_.z)),
                ros::Time::now(), "map", "base"));

        loopRate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map_merge");
    ros::NodeHandle nh;
    std::string global_map_path;
    std::string config_path;
    std::string bag_path;
    std::string imu_topic;
    std::string lidar_topic;
    double min_distance;
    double max_distance;
    bool imu_init;
    bool lidar_init;
    int bag_param;
    sensor_msgs::PointCloud2 TargetCloud;
    sensor_msgs::PointCloud2 TargetCloud2;
    geometry_msgs::PointStamped global_origin;
    geometry_msgs::PolygonStamped empty_polygon;

    nh.getParam("global_map_path", global_map_path);
    nh.getParam("config_path", config_path);
    nh.getParam("bag_path", bag_path);
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("min_distance", min_distance);
    nh.getParam("max_distance", max_distance);
    nh.getParam("bag_param", bag_param);

    ros::Rate loop_rate(1);
    ros::Publisher occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/ogm_json", 1);
    ros::Publisher merged_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/merged_map", 2);
    ros::Publisher local_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 2);
    ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 2);
    ros::Publisher filteredcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filteredcloud", 2);
    ros::Publisher global_pub = nh.advertise<geometry_msgs::PointStamped>("/global_origin", 1);
    ros::Publisher local_pub = nh.advertise<geometry_msgs::PointStamped>("/local_origin", 1);
    ros::Publisher empty_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("/empty_polygon", 1, true);

    nav_msgs::OccupancyGrid global_map;

    Json::Reader json_reader;
    std::ifstream json_file;
    Json::Value root;

    imu_init = false;
    lidar_init = false;
    pLaserCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    FilteredCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    json_file.open(global_map_path);

    if (!json_reader.parse(json_file, root))
    {
        std::cout << "Error opening json file : " << global_map_path << std::endl;
    }

    int width, height, size;
    float resolution;
    o_global_map_.header.frame_id = "/map";
    o_global_map_.info.width = root["width"].asInt();
    o_global_map_.info.height = root["height"].asInt();
    o_global_map_.info.resolution = root["resolution"].asFloat();
    o_global_map_.info.origin.position.x = root["origin.x"].asDouble();
    o_global_map_.info.origin.position.y = root["origin.y"].asDouble();
    o_global_map_.info.origin.position.z = 0.0;
    global_map.header.frame_id = "/map";
    global_map.info.width = root["width"].asInt();
    global_map.info.height = root["height"].asInt();
    global_map.info.resolution = root["resolution"].asFloat();
    global_map.info.origin.position.x = root["origin.x"].asDouble();
    global_map.info.origin.position.y = root["origin.y"].asDouble();
    global_map.info.origin.position.z = 0.0;
    global_map_heading = root["origin.heading"].asFloat();
    size = root["data"].size();

    Json::Reader o_json_reader;
    std::ifstream o_json_file;
    Json::Value o_root;

    o_json_file.open(config_path);

    if (!o_json_reader.parse(o_json_file, o_root))
    {
        std::cout << "Error opening json file : " << config_path << std::endl;
    }
    d_width1 = o_root["d_width1"].asDouble();
    d_width2 = o_root["d_width2"].asDouble();
    d_height1 = o_root["d_height1"].asDouble();
    d_height2 = o_root["d_height2"].asDouble();
    d_z1 = o_root["d_z1"].asDouble();
    d_z2 = o_root["d_z2"].asDouble();
    d_ego_left = o_root["d_ego_left"].asDouble();
    d_ego_right = o_root["d_ego_right"].asDouble();
    d_ego_front = o_root["d_ego_front"].asDouble();
    d_ego_back = o_root["d_ego_back"].asDouble();
    d_ego_top = o_root["d_ego_top"].asDouble();
    d_ego_bottom = o_root["d_ego_bottom"].asDouble();
    d_resolution = o_root["d_resolution"].asDouble();
    d_height_diff = o_root["d_height_diff"].asDouble();
    i_max_thresh = o_root["i_classify_max_thresh"].asInt();
    i_mid_thresh = o_root["i_classify_mid_thresh"].asInt();
    i_min_thresh = o_root["i_classify_min_thresh"].asInt();
    i_classify_park_thresh = o_root["i_classify_park_thresh"].asInt();
    d_ego_empty_left = o_root["d_ego_empty_left"].asDouble();
    d_ego_empty_right = o_root["d_ego_empty_right"].asDouble();
    d_ego_empty_front = o_root["d_ego_empty_front"].asDouble();
    d_ego_empty_back = o_root["d_ego_empty_back"].asDouble();
    d_park_height = o_root["d_park_height"].asDouble();

    for (int i = 0; i < size; i++)
    {
        o_global_map_.data.push_back(root["data"][i].asInt());
        global_map.data.push_back(root["data"][i].asInt());
    }

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(lidar_topic));
    topics.push_back(std::string(imu_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it = view.begin();
    std::advance(it, bag_param);
    // 按绝对的时间顺序读取
    for (; it != view.end(); ++it)
    {
        auto m = *it;
        std::string topic = m.getTopic();
        if (topic == std::string(lidar_topic) && !lidar_init)
        {
            sensor_msgs::PointCloud2::ConstPtr pcloud = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
            if (pcloud != NULL)
            {
                pcl::fromROSMsg(*pcloud, *tmp);
                pointcloud_in_range(min_distance, max_distance, tmp, pLaserCloud);
                lidar_init = true;
            }
            // for visualization
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(pLaserCloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(d_width1 + 1, d_width2 - 1);
            // pass.setFilterLimits(1, 10);
            pass.setFilterLimitsNegative(false);
            pass.filter(*FilteredCloud);
            pass.setInputCloud(FilteredCloud);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(d_height1 + 1, d_height2 - 1);
            // pass.setFilterLimits(-5, -4);
            pass.setFilterLimitsNegative(false);
            pass.filter(*FilteredCloud);
            pass.setInputCloud(FilteredCloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(d_z1, d_z2);
            pass.setFilterLimitsNegative(false);
            pass.filter(*FilteredCloud);
            std::cout << "before size:  " << pLaserCloud->size() << std::endl;
            std::cout << "fliter size:  " << FilteredCloud->size() << std::endl;
        }
        else if (topic == std::string(imu_topic) && !imu_init)
        {
            sensor_msgs::Imu::ConstPtr pRtk = m.instantiate<sensor_msgs::Imu>();
            if (pRtk != NULL)
            {
                o_imu_msg_.AngleHeading = pRtk->orientation.w;
                transform_llh_to_xyz(pRtk->orientation.y, pRtk->orientation.x, pRtk->orientation.z, o_imu_msg_.x, o_imu_msg_.y, o_imu_msg_.z);
                if (pRtk->orientation_covariance[2] == 4)
                {
                    imu_init = true;
                }
            }
        }
        if (imu_init && lidar_init)
        {
            pcl::toROSMsg(*pLaserCloud, TargetCloud);
            TargetCloud.header.stamp = ros::Time::now();
            TargetCloud.header.frame_id = "base";
            CreateLocalMap();
            UpdateGloballMap();
            pcl::toROSMsg(*FilteredCloud, TargetCloud2);
            TargetCloud2.header.stamp = ros::Time::now();
            TargetCloud2.header.frame_id = "base";
            break;
        }
    }

    global_origin.header.frame_id = "/map";
    global_origin.header.stamp = ros::Time::now();
    global_origin.point.x = global_map.info.origin.position.x;
    global_origin.point.y = global_map.info.origin.position.y;

    empty_polygon.header.frame_id = "base";
    geometry_msgs::Point32 A;
    geometry_msgs::Point32 B;
    geometry_msgs::Point32 C;
    geometry_msgs::Point32 D;
    A.y = d_ego_empty_left;
    A.x = d_ego_empty_front;
    B.y = d_ego_empty_right;
    B.x = d_ego_empty_front;
    C.y = d_ego_empty_right;
    C.x = d_ego_empty_back;
    D.y = d_ego_empty_left;
    D.x = d_ego_empty_back;

    empty_polygon.polygon.points.push_back(A);
    empty_polygon.polygon.points.push_back(B);
    empty_polygon.polygon.points.push_back(C);
    empty_polygon.polygon.points.push_back(D);

    std::thread publishTf(publishTfFcn);
    publishTf.detach();

    while (ros::ok())
    {
        o_global_map_.header.stamp = ros::Time::now();
        global_map.header.stamp = ros::Time::now();
        TargetCloud.header.stamp = ros::Time::now();
        TargetCloud2.header.stamp = ros::Time::now();
        o_local_map_.header.stamp = ros::Time::now();
        empty_polygon.header.stamp = ros::Time::now();

        occupancy_grid_pub.publish(global_map);
        merged_grid_pub.publish(o_global_map_);
        pointcloud_pub.publish(TargetCloud);
        filteredcloud_pub.publish(TargetCloud2);
        local_grid_pub.publish(o_local_map_);
        global_pub.publish(global_origin);
        local_pub.publish(local_origin);
        empty_polygon_pub.publish(empty_polygon);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
