/*
 * @Descripttion: 
 * @version: 
 * @Author: Yuhang Li
 * @Date: 2022-01-12 09:26:38
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-01-29 10:34:12
 */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <json/json.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

#define foreach BOOST_FOREACH
#define GLOBAL_ORIGIN_LAT 31.1367846
#define GLOBAL_ORIGIN_LON 118.1789369

class OGM
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    OGM(ros::NodeHandle nh) : nh_(nh)
    {
        nh_.getParam("map_path", map_path);
        nh_.getParam("map_resolution", map_resolution);
        nh_.getParam("thre_z_min", thre_z_min);
        nh_.getParam("thre_z_max", thre_z_max);
        nh_.getParam("height_threshold", height_threshold);
        nh_.getParam("flag_pass_through", flag_pass_through);
        nh_.getParam("filter_radius", filter_radius);
        nh_.getParam("filter_count", filter_count);
        nh_.getParam("count_threshold", count_threshold);
        // 在原本地图大小(由有点的最小最大xy决定)上向四周扩大inflation_map_x 和 inflation_map_y 尺寸
        nh_.getParam("inflation_map_x", inflation_map_x);
        nh_.getParam("inflation_map_y", inflation_map_y);
        nh_.getParam("bag_path", bag_path);
        nh_.getParam("get_map_origin_informaiton", get_map_origin_informaiton);
        nh_.getParam("imu_topic", imu_topic);
        nh_.getParam("pre_json_path", pre_json_path);
        nh_.getParam("map_update", map_update);
        nh_.getParam("visualize", visualize);
        map_pcd_path = map_path + "/map.pcd";
        filtered_pcd_path = map_path + "/filtered_map.pcd";
        saved_json_path = map_path + "/global_map.json";
        transformed_pcd_path = map_path + "/transformed_map.pcd";
        // nh_.getParam("filtered_pcd_path", filtered_pcd_path);
        // nh_.getParam("saved_json_path", saved_json_path);
    }
    void create_ogm()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        nav_msgs::OccupancyGrid occupancy_grid_map;
        nav_msgs::OccupancyGrid inflated_occupancy_grid_map;
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_pcd_path, *pcd_cloud) == -1)
        {
            PCL_ERROR("Could not read file :  %s \n", map_pcd_path.c_str());
            return;
        }
        PointCloudFilter(thre_z_min, thre_z_max, flag_pass_through, pcd_cloud, filtered_cloud);
        pcl::io::savePCDFileASCII(filtered_pcd_path, *pcd_cloud);
        pcl::io::savePCDFileASCII(transformed_pcd_path, *transform_origin(pcd_cloud));
        SetOccupancyGridMap(filtered_cloud, occupancy_grid_map, inflated_occupancy_grid_map, map_update);
        printf("occupancy grid map created ~~~ \n");
        printf("start creating json file .... \n");
        Json::Value root;

        for (int i = 0; i < (inflated_occupancy_grid_map_width * inflated_occupancy_grid_map_height); i++)
        {
            root["data"].append(inflated_occupancy_grid_map.data[i]);
        }
        root["header.frame_id"] = Json::Value("map");
        root["header.stamp"] = Json::Value(0.0);
        root["origin.x"] = Json::Value(inflated_origin_x);
        root["origin.y"] = Json::Value(inflated_origin_y);
        root["origin.z"] = Json::Value(0.0);
        root["origin.heading"] = Json::Value(0.0);
        root["width"] = Json::Value(inflated_occupancy_grid_map_width);
        root["height"] = Json::Value(inflated_occupancy_grid_map_height);
        root["length"] = Json::Value(20.0);
        root["resolution"] = Json::Value(1.0);

        Json::StyledWriter sw;
        std::ofstream os;
        os.open(saved_json_path, std::ios::out);
        if (!os.is_open())
            std::cout << "error:can not find or create the file which named \" demo.json\"." << std::endl;
        os << sw.write(root);
        os.close();
        printf("created json file !!!! \n");
    }

    void SetOccupancyGridMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, nav_msgs::OccupancyGrid &msg, nav_msgs::OccupancyGrid &inflated_msg, bool &map_update)
    {
        msg.header.seq = 0;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";

        msg.info.map_load_time = ros::Time::now();
        msg.info.resolution = map_resolution;

        double x_min, x_max, y_min, y_max, z_min, z_max;

        if (cloud->points.empty())
        {
            ROS_WARN("pcd is empty!\n");

            return;
        }

        //calculate the max and min x and y
        for (int i = 0; i < cloud->points.size() - 1; i++)
        {
            if (i == 0)
            {
                x_min = x_max = cloud->points[i].x;
                y_min = y_max = cloud->points[i].y;
                z_min = z_max = cloud->points[i].z;
            }

            double x = cloud->points[i].x;
            double y = cloud->points[i].y;
            double z = cloud->points[i].z;

            if (x < x_min)
                x_min = x;
            if (x > x_max)
                x_max = x;

            if (y < y_min)
                y_min = y;
            if (y > y_max)
                y_max = y;

            if (z < z_min)
                z_min = z;
            if (z > z_max)
                z_max = z;
        }

        std::cout << "x_min is : " << x_min << std::endl;
        std::cout << "x_max is : " << x_max << std::endl;
        std::cout << "y_min is : " << y_min << std::endl;
        std::cout << "y_max is : " << y_max << std::endl;
        std::cout << "z_min is : " << z_min << std::endl;
        std::cout << "z_max is : " << z_max << std::endl;
        //transform the origin of map to the x_min and y_min
        msg.info.origin.position.x = origin_x_in_world + x_min;
        msg.info.origin.position.y = origin_y_in_world + y_min;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.info.width = int((x_max - x_min) / map_resolution);
        msg.info.height = int((y_max - y_min) / map_resolution);
        occupancy_grid_map_width = msg.info.width;
        occupancy_grid_map_height = msg.info.height;

        std::cout << "The width is : " << msg.info.width << std::endl;
        std::cout << "The height is : " << msg.info.height << std::endl;
        //initialize the two vectors
        std::vector<std::vector<float>> maxheight(msg.info.height);
        std::vector<std::vector<float>> minheight(msg.info.height);
        std::vector<std::vector<float>> count(msg.info.height);

        for (int i = 0; i < msg.info.height; i++)
        {
            maxheight[i].resize(msg.info.width);
            minheight[i].resize(msg.info.width);
            count[i].resize(msg.info.width);

            for (int j = 0; j < msg.info.width; j++)
            {
                maxheight[i][j] = std::numeric_limits<float>::min();
                minheight[i][j] = std::numeric_limits<float>::max();
                count[i][j] = 0;
            }
        }

        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (cloud->points[i].x > x_min + 1.1 && cloud->points[i].x < x_max - 1.1 && cloud->points[i].y > y_min + 1.1 && cloud->points[i].y < y_max - 1.1)
            {
                int x = int((cloud->points[i].x - x_min) / map_resolution);
                int y = int((cloud->points[i].y - y_min) / map_resolution);

                count[y][x]++;
                if (cloud->points[i].z > maxheight[y][x])
                {
                    maxheight[y][x] = cloud->points[i].z;
                }
                if (cloud->points[i].z < minheight[y][x])
                {
                    minheight[y][x] = cloud->points[i].z;
                }
            }
        }
        msg.data.resize(msg.info.width * msg.info.height);
        msg.data.assign(msg.info.width * msg.info.height, 0);
        occupancy_grid_map_data.resize(msg.info.width * msg.info.height);
        occupancy_grid_map_data.assign(msg.info.width * msg.info.height, 0);

        std::cout << "THE MAP SIZE IS : " << msg.data.size() << std::endl;

        for (int i = 0; i < msg.info.height; i++)
        {
            for (int j = 0; j < msg.info.width; j++)
            {
                if (maxheight[i][j] - minheight[i][j] > height_threshold && count[i][j] > count_threshold)
                {
                    int index = j + i * msg.info.width;
                    msg.data[index] = 100; //(maxheight[i][j] - minheight[i][j]) * 100;
                    occupancy_grid_map_data[index] = msg.data[index];
                }
            }
        }

        cv::Mat ogm_mat = cv::Mat(msg.info.height, msg.info.width, CV_8UC1);
        for (int i = 0; i < msg.info.height; i++)
        {
            for (int j = 0; j < msg.info.width; j++)
            {
                int index = j + i * msg.info.width;
                ogm_mat.at<uint8_t>(i, j) = occupancy_grid_map_data[index];
            }
        }
        if (visualize)
        {
            //if you want to visualize the mat ,please uncomment these
            cv::imshow("原始图", ogm_mat);
            cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 3));
            // cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
            cv::erode(ogm_mat, ogm_mat, erode_element);
            cv::imshow("腐蚀图", ogm_mat);
            cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
            cv::dilate(ogm_mat, ogm_mat, dilate_element);
            cv::namedWindow("膨胀图", CV_WINDOW_NORMAL);
            cv::imshow("膨胀图", ogm_mat);
            cv::waitKey(0);
        }

        for (int i = 0; i < msg.info.height; i++)
        {
            for (int j = 0; j < msg.info.width; j++)
            {
                int index = j + i * msg.info.width;
                occupancy_grid_map_data[index] = (ogm_mat.at<uint8_t>(i, j) > 100) ? 100 : ogm_mat.at<uint8_t>(i, j);
            }
        }

        //inflate the occupancy grid map
        inflated_msg.header.seq = 0;
        inflated_msg.header.stamp = ros::Time::now();
        inflated_msg.header.frame_id = "map";

        inflated_msg.info.map_load_time = ros::Time::now();
        inflated_msg.info.resolution = map_resolution;

        inflated_msg.info.origin.position.x = origin_x_in_world + x_min - inflation_map_x;
        inflated_msg.info.origin.position.y = origin_y_in_world + y_min - inflation_map_y;
        inflated_msg.info.origin.position.z = 0.0;
        inflated_msg.info.origin.orientation.x = 0.0;
        inflated_msg.info.origin.orientation.y = 0.0;
        inflated_msg.info.origin.orientation.z = 0.0;
        inflated_msg.info.origin.orientation.w = 1.0;

        inflated_origin_x = inflated_msg.info.origin.position.x;
        inflated_origin_y = inflated_msg.info.origin.position.y;

        inflated_msg.info.width = int((x_max - x_min) / map_resolution + 2 * inflation_map_x);
        inflated_msg.info.height = int((y_max - y_min) / map_resolution + 2 * inflation_map_y);
        inflated_occupancy_grid_map_width = inflated_msg.info.width;
        inflated_occupancy_grid_map_height = inflated_msg.info.height;

        inflated_msg.data.resize(inflated_msg.info.width * inflated_msg.info.height);
        inflated_msg.data.assign(inflated_msg.info.width * inflated_msg.info.height, 0);
        // inflated_occupancy_grid_map_data.resize(inflated_msg.info.width * inflated_msg.info.height);
        // inflated_occupancy_grid_map_data.assign(inflated_msg.info.width * inflated_msg.info.height, 0);

        if (map_update)
        {
            Json::Reader json_reader;
            std::ifstream json_file;
            json_file.open(pre_json_path);
            Json::Value root;
            nav_msgs::OccupancyGrid pre_saved_map;
            int size;

            if (!json_reader.parse(json_file, root))
            {
                std::cout << "Error opening prejson file : " << pre_json_path << std::endl;
            }
            pre_saved_map.info.width = root["width"].asInt();
            pre_saved_map.info.height = root["height"].asInt();
            pre_saved_map.info.resolution = root["resolution"].asFloat();
            pre_saved_map.header.frame_id = "/map";
            pre_saved_map.info.origin.position.x = root["origin.x"].asDouble();
            pre_saved_map.info.origin.position.y = root["origin.y"].asDouble();
            pre_saved_map.info.origin.position.z = 0.0;
            size = root["data"].size();

            for (int i = 0; i < size; i++)
            {
                pre_saved_map.data.push_back(root["data"][i].asInt());
            }
            std::cout << "inflated_origin_x : " << inflated_origin_x << std::endl;
            std::cout << "inflated_origin_y : " << inflated_origin_y << std::endl;
            std::cout << "inflated_occupancy_grid_map_width : " << inflated_occupancy_grid_map_width << std::endl;
            std::cout << "inflated_occupancy_grid_map_height : " << inflated_occupancy_grid_map_height << std::endl;
            std::cout << "pre_saved_map.info.origin.position.x : " << pre_saved_map.info.origin.position.x << std::endl;
            std::cout << "pre_saved_map.info.origin.position.y : " << pre_saved_map.info.origin.position.y << std::endl;
            std::cout << "pre_saved_map.info.height : " << pre_saved_map.info.height << std::endl;
            std::cout << "pre_saved_map.info.width : " << pre_saved_map.info.width << std::endl;

            // cv::Mat pre_mat = cv::Mat(inflated_msg.info.height, inflated_msg.info.width, CV_8UC1);
            for (int i = 0; i < pre_saved_map.info.height; i++)
            {
                for (int j = 0; j < pre_saved_map.info.width; j++)
                {
                    int index = j + i * pre_saved_map.info.width;
                    if (pre_saved_map.info.origin.position.x + j >= inflated_origin_x && pre_saved_map.info.origin.position.x + j < inflated_origin_x + inflated_occupancy_grid_map_width - 1 &&
                        pre_saved_map.info.origin.position.y + i >= inflated_origin_y && pre_saved_map.info.origin.position.y + i < inflated_origin_y + inflated_occupancy_grid_map_height - 1)
                    {
                        int new_index = int(j + pre_saved_map.info.origin.position.x - inflated_origin_x) + int(i + pre_saved_map.info.origin.position.y - inflated_origin_y) * inflated_occupancy_grid_map_width;
                        // std::cout << "mapy : " << i + pre_saved_map.info.origin.position.y - inflated_origin_y << std::endl;
                        // std::cout << "mapx : " << j + pre_saved_map.info.origin.position.x - inflated_origin_x << std::endl;
                        inflated_msg.data[new_index] = pre_saved_map.data[index];
                        // pre_mat.at<uint8_t>(int(i + pre_saved_map.info.origin.position.y - inflated_origin_y), int(j + pre_saved_map.info.origin.position.x - inflated_origin_x)) = pre_saved_map.data[index];
                    }
                }
            }

            if (visualize)
            {
                cv::Mat pre_mat_origin = cv::Mat(pre_saved_map.info.height, pre_saved_map.info.width, CV_8UC1);
                for (int i = 0; i < pre_saved_map.info.height; i++)
                {
                    for (int j = 0; j < pre_saved_map.info.width; j++)
                    {
                        int index = j + i * pre_saved_map.info.width;
                        pre_mat_origin.at<uint8_t>(i, j) = pre_saved_map.data[index];
                    }
                }
                cv::Mat pre_mat = cv::Mat(inflated_msg.info.height, inflated_msg.info.width, CV_8UC1);
                for (int i = 0; i < inflated_msg.info.height; i++)
                {
                    for (int j = 0; j < inflated_msg.info.width; j++)
                    {
                        int index = j + i * inflated_msg.info.width;
                        pre_mat.at<uint8_t>(i, j) = inflated_msg.data[index];
                    }
                }
                cv::namedWindow("上一次地图", CV_WINDOW_NORMAL);
                cv::imshow("上一次地图", pre_mat_origin);
                cv::namedWindow("上一次地图转换后", CV_WINDOW_NORMAL);
                cv::imshow("上一次地图转换后", pre_mat);
                cv::waitKey(0);
            }
        }

        for (int i = 0; i < msg.info.height; i++)
        {
            for (int j = 0; j < msg.info.width; j++)
            {
                int new_index = (j + inflation_map_x) + (i + inflation_map_y) * inflated_occupancy_grid_map_width;
                int index = j + i * msg.info.width;
                inflated_msg.data[new_index] = occupancy_grid_map_data[index];
            }
        }
        if (visualize)
        {
            cv::Mat afr_mat = cv::Mat(inflated_msg.info.height, inflated_msg.info.width, CV_8UC1);
            for (int i = 0; i < inflated_msg.info.height; i++)
            {
                for (int j = 0; j < inflated_msg.info.width; j++)
                {
                    int index = j + i * inflated_msg.info.width;
                    afr_mat.at<uint8_t>(i, j) = inflated_msg.data[index];
                }
            }
            cv::namedWindow("更新后地图", CV_WINDOW_NORMAL);
            cv::imshow("更新后地图", afr_mat);
            cv::waitKey(0);
        }
    }

    void PointCloudFilter(const double &thre_low, const double &thre_high, const bool &flag_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
    {
        cloud_out->clear();

        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(30);
        sor.setStddevMulThresh(0.2);
        sor.filter(*cloud_out);
    }

    void getOriginXY()
    {
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            std::string topic = m.getTopic();
            sensor_msgs::Imu::ConstPtr pRtk = m.instantiate<sensor_msgs::Imu>();
            if (!get_map_origin_informaiton)
            {
                origin_latitude = pRtk->orientation.x;
                origin_longitude = pRtk->orientation.y;
                origin_altitude = pRtk->orientation.z;
                std::cout << "origin_latitude is : " << std::setprecision(10) << origin_latitude << std::endl;
                std::cout << "origin_lontitude is : " << std::setprecision(10) << origin_longitude << std::endl;
                std::cout << "origin_altitude is : " << std::setprecision(10) << origin_altitude << std::endl;

                transform_llh_to_xyz(origin_longitude, origin_latitude, origin_altitude, origin_x_in_world, origin_y_in_world, origin_z_in_world);
                std::cout << "origin_x is : " << origin_x_in_world << std::endl;
                std::cout << "origin_y is : " << origin_y_in_world << std::endl;
                get_map_origin_informaiton = true;
            }
        }
        bag.close();

        return;
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

    pcl::PointCloud<pcl::PointXYZI>::Ptr transform_origin(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_out->resize(cloud_in->points.size());

        for (uint32_t i = 0; i < cloud_in->points.size(); i++)
        {
            Eigen::Vector3d point(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);

            pcl::PointXYZI p;
            p.x = point[0] + origin_x_in_world;
            p.y = point[1] + origin_y_in_world;
            p.z = point[2] + origin_z_in_world;
            p.intensity = cloud_in->points[i].intensity;

            cloud_out->points[i] = p;
        }

        cloud_out->height = 1;
        cloud_out->width = cloud_in->points.size();

        return cloud_out;
    }

private:
    ros::NodeHandle nh_;

    std::string map_path;
    std::string map_pcd_path;
    std::string bag_path;
    std::string imu_topic;
    std::string filtered_pcd_path;
    std::string transformed_pcd_path;
    std::string pre_json_path;
    bool map_update;
    bool visualize;

    double map_resolution;
    double thre_z_min;
    double thre_z_max;
    int flag_pass_through;
    int occupancy_grid_map_width;
    int occupancy_grid_map_height;
    int occupancy_grid_map_length;
    std::vector<uint8_t> occupancy_grid_map_data;
    std::vector<uint8_t> inflated_occupancy_grid_map_data;
    int inflated_occupancy_grid_map_width;
    int inflated_occupancy_grid_map_height;
    bool get_map_origin_information;
    double origin_x_in_world;
    double origin_y_in_world;
    double origin_z_in_world;
    double inflated_origin_x;
    double inflated_origin_y;
    double height_threshold;
    std::string saved_json_path;
    double filter_radius;
    int filter_count;
    int count_threshold;
    int inflation_map_x;
    int inflation_map_y;
    bool get_map_origin_informaiton;

    double origin_latitude;
    double origin_longitude;
    double origin_altitude;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_ogm");
    ros::NodeHandle nh;
    OGM ogm(nh);

    ogm.getOriginXY();
    ogm.create_ogm();

    return 0;
}