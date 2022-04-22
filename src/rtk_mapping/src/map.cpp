/*
 * @Descripttion: 
 * @version: 
 * @Author: Yuhang Li
 * @Date: 2022-01-12 09:26:38
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-01-22 16:30:00
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <boost/foreach.hpp>
#include <unistd.h>
#include <string>

#include <mutex>
#include <thread>
#include <chrono>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "func.h"
#include "tictoc.h"
#include "Scancontext.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <rtk_mapping/node.h>
#include <nav_msgs/OccupancyGrid.h>
#include <json/json.h>
#include <fstream>

using namespace gtsam;

#define foreach BOOST_FOREACH

class MapOptimization
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    MapOptimization(ros::NodeHandle nh) : nh_(nh)
    {
        // load params from launch
        nh_.getParam("bag_path", bag_path);
        nh_.getParam("map_path", map_path);
        nh_.getParam("save_map", save_map);
        nh_.getParam("update_map", update_map);
        nh_.getParam("map_dist", map_dist);
        nh_.getParam("submap_size", submap_size);
        nh_.getParam("save_map_dist", save_map_dist);

        //load occupancy grid map params
        nh_.getParam("min_distance", min_distance);
        nh_.getParam("max_distance", max_distance);
        nh_.getParam("get_map_origin_informaiton", get_map_origin_informaiton);
        nh_.getParam("lidar_topic", lidar_topic);
        nh_.getParam("imu_topic", imu_topic);
        nh_.getParam("radius_filter_radius", filter_radius);
        nh_.getParam("radius_filter_count", filter_count);
        nh_.getParam("optimized_pose_path", optimized_pose_path);
        nh_.getParam("former_bag_path", former_bag_path);
        // nh_.getParam("saved_json_path", saved_json_path);
        // nh_.getParam("map_pcd_path", map_pcd_path);
        // nh_.getParam("final_map_path", final_map_path);

        sc_path = map_path + "/SCDs";
        key_frames_path = map_path + "/key_frames";
        dense_key_frames_path = map_path + "/dense_key_frames";
        // saved_json_path = map_path + "/global_map.json";
        // map_pcd_path = map_path + "/original_map.pcd";
        final_map_path = map_path + "/map.pcd";
        original_map_pcd = map_path + "/original_map.pcd";
        dense_map_pcd = map_path + "/dense_map.pcd";
        submap_path = map_path + "/submap";
        scans_path = map_path + "/Scans";
        update_map_path = map_path + "/update";
        optimized_pose_path = map_path + "/update/out/" + optimized_pose_path;

        if (update_map)
        {
            cout << "update map!!!!!!!!!" << endl;
            final_map_path = update_map_path + "/updated_map.pcd";
            original_map_pcd = update_map_path + "/original_map.pcd";
            dense_map_pcd = update_map_path + "/dense_map.pcd";
        }

        //ros
        pubMap = nh.advertise<sensor_msgs::PointCloud2>("/map", 2);
        pubOdom = nh.advertise<geometry_msgs::PoseStamped>("/odometry", 5);
        pubPath = nh.advertise<nav_msgs::Path>("/path", 1);
        pubNode = nh.advertise<rtk_mapping::node>("/node", 5);
        pubrtk = nh.advertise<geometry_msgs::PoseStamped>("rtk_odom", 5);
        pubRtkPath = nh.advertise<nav_msgs::Path>("/rtk_path", 1);
        pubRtkGt = nh.advertise<nav_msgs::Path>("/rtk_gt", 1);
        pubOccupancyGridMap = nh.advertise<nav_msgs::OccupancyGrid>("/ogm", 1);

        /* -------------------------------------------------------------------------- */
        /*                      Extrinsic Matrix of Lidar and RTK                     */
        /* -------------------------------------------------------------------------- */

        R_imu2lidar = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * //-M_PI / 2
                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        // R_imu2lidar <<  -0.8553 , 0.517842, -0.0173789,
        //                 -0.518131 , -0.85492 , 0.0255567,
        //                 -0.00162325, 0.0308632, 0.999522;
        q_imu2lidar = R_imu2lidar;
        q_lidar2imu = R_imu2lidar.transpose();
        t_lidar_2_imu = Eigen::Vector3d(0.0, 0.0, 0.0);
        t_imu_2_lidar = Eigen::Vector3d(0.0, 0.0, 0.0);
        // t_lidar_2_imu = Eigen::Vector3d(-2.52511911, 7.49338716, 1.76564517);
        // t_imu_2_lidar = Eigen::Vector3d(-6.00944, 5.05278, -2.00017);

        //pointcloud init
        pLaserCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pLaserCloudDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pMapCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pMapCloudDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pTargetCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pSourceCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pRSTargetCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_after_rotation.reset(new pcl::PointCloud<pcl::PointXYZI>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

        //pose init
        KeyFramePoses3D.reset(new pcl::PointCloud<pcl::PointXYZI>());
        rtkKeyFramePoses3D.reset(new pcl::PointCloud<pcl::PointXYZI>());
        q_lidar_rtk_last = Eigen::Quaterniond(1, 0, 0, 0);
        t_lidar_rtk_last = Eigen::Vector3d(0, 0, 0);

        q_lidar_rtk = Eigen::Quaterniond(1, 0, 0, 0);
        t_lidar_rtk = Eigen::Vector3d(0, 0, 0);

        q_odometry = Eigen::Quaterniond(1, 0, 0, 0);
        t_odometry = Eigen::Vector3d(0, 0, 0);

        q_sam = Eigen::Quaterniond(1, 0, 0, 0);
        t_sam = Eigen::Vector3d(0, 0, 0);

        currentPose.x = 0;
        currentPose.y = 0;
        currentPose.z = 0;
        currentPose.intensity = 0;

        //Voxel grid init
        downSizeFilter_50.setLeafSize(0.5, 0.5, 0.5);
        downSizeFilter_25.setLeafSize(0.25, 0.25, 0.25);

        //gtsam
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        gtsam::Vector Vector1(6);
        Vector1 << 1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3; // rotation xyz
        odometryNoise = noiseModel::Diagonal::Variances(Vector1);

        gtsam::Vector Vector2(6);
        Vector2 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
        priorNoise = noiseModel::Diagonal::Variances(Vector2);

        //ndt
        anh_ndt.setResolution(1.0);
        anh_ndt.setMaximumIterations(50);
        anh_ndt.setStepSize(0.1);
        anh_ndt.setTransformationEpsilon(0.01);

        //other init
        odometry_time = 0;
        loopClosure_time = 0;
        latestFrameID = -1;
        detected_history_keyframe_id = -1;
        dist = map_dist;

        //manul params
        // surroundingKeyframeSearchRadius = 25;
        surroundingKeyframeSearchRadius = 15;
        recentKeyFramesNum = 10;

        //debug
        close_index = 0;
        lidar_topic_num = 0;
        lidar_topic_count = 0;

        if (!update_map)
        {
            pgTimeSaveStream = std::fstream(map_path + "/" + "times.txt", std::fstream::out);
            pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

            pgSaveStream = std::fstream(map_path + "/singlesession_posegraph.g2o", std::fstream::out);
            rtkSaveStream = std::fstream(map_path + "/singlesession_rtkpose.g2o", std::fstream::out);

            if (access(map_path.c_str(), 6) != 0)
            {
                int map_create = mkdir(map_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                if (!map_create)
                    printf("create map_path:%s\n", map_path.c_str());
                else
                    printf("create map_path failed! error code : %d \n", map_create);
            }
            if (access(sc_path.c_str(), 6) != 0)
            {
                int sc_create = mkdir(sc_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                if (!sc_create)
                    printf("create sc_path:%s\n", sc_path.c_str());
                else
                    printf("create sc_path failed! error code : %d \n", sc_create);
            }
            if (access(key_frames_path.c_str(), 6) != 0)
            {
                int key_frames_create = mkdir(key_frames_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                if (!key_frames_create)
                    printf("create key_frames_path:%s\n", key_frames_path.c_str());
                else
                    printf("create key_frames_path failed! error code : %d \n", key_frames_create);
            }
            if (access(dense_key_frames_path.c_str(), 6) != 0)
            {
                int dense_key_frames_create = mkdir(dense_key_frames_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                if (!dense_key_frames_create)
                    printf("create dense_key_frames_path:%s\n", dense_key_frames_path.c_str());
                else
                    printf("create dense_key_frames_path failed! error code : %d \n", dense_key_frames_create);
            }
            if (access(scans_path.c_str(), 6) != 0)
            {
                int scans_create = mkdir(scans_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                if (!scans_create)
                    printf("create scans_path:%s\n", scans_path.c_str());
                else
                    printf("create scans_path failed! error code : %d \n", scans_create);
            }

            // if (access(submap_path.c_str(), 6) != 0)
            // {
            //     int submap_create = mkdir(submap_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
            //     if (!submap_create)
            //         printf("create submap_path:%s\n", submap_path.c_str());
            //     else
            //         printf("create submap_path failed! error code : %d \n", submap_create);
            // }
        }
        else
        {
            if (access(update_map_path.c_str(), 6) != 0)
            {
                int update_map_create = mkdir(update_map_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
                if (!update_map_create)
                    printf("create update_map_path:%s\n", update_map_path.c_str());
                else
                    printf("create update_map_path failed! error code : %d \n", update_map_create);
            }
        }
    }

    void getRtkPose()
    {
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            // std::string topic = m.getTopic();

            sensor_msgs::Imu::ConstPtr pRtk = m.instantiate<sensor_msgs::Imu>();
            if (!get_map_origin_informaiton)
            {
                origin_heading = pRtk->orientation.w;
                std::cout << "The heading of origin is : " << origin_heading << std::endl;
                get_map_origin_informaiton = true;
            }
            if (pRtk != NULL)
                pose_manager.rtk_data_in(*pRtk);
        }
        bag.close();
        if (update_map)
        {
            // 考虑到第一帧建图时已经有t（yaw）  一次按central sess的坐标系旋转，不必再一次旋转第一帧的yaw 就可使第一帧朝向正北
            bag.open(former_bag_path, rosbag::bagmode::Read);
            rosbag::View former_view(bag, rosbag::TopicQuery(topics));
            rosbag::View::iterator it = former_view.begin();
            for (; it != former_view.end(); ++it)
            {
                auto m = *it;
                sensor_msgs::Imu::ConstPtr pRtk = m.instantiate<sensor_msgs::Imu>();
                if (pRtk != NULL)
                {
                    origin_heading = pRtk->orientation.w;
                    std::cout << "[revised]: The heading of origin is : " << pRtk->orientation.w << std::endl;
                    break;
                }
            }
            bag.close();
        }
    }

    void writeVertexFromQT(const int _node_idx, const Eigen::Quaternion<double> &q_, const Eigen::Vector3d &t_)
    {
        std::string curVertexInfo{
            "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " " + std::to_string(t_(0)) + " " + std::to_string(t_(1)) + " " + std::to_string(t_(2)) + " " + std::to_string(q_.x()) + " " + std::to_string(q_.y()) + " " + std::to_string(q_.z()) + " " + std::to_string(q_.w())};

        // pgVertexSaveStream << curVertexInfo << std::endl;
        vertices_str.emplace_back(curVertexInfo);
    }

    void writeVertex(const int _node_idx, const gtsam::Pose3 &_initPose)
    {
        gtsam::Point3 t = _initPose.translation();
        gtsam::Rot3 R = _initPose.rotation();

        //点序号 xyz xyzw
        std::string curVertexInfo{
            "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " " + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z()) + " " + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w())};

        // pgVertexSaveStream << curVertexInfo << std::endl;
        vertices_str.emplace_back(curVertexInfo);
    }

    void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3 &_relPose)
    {
        gtsam::Point3 t = _relPose.translation();
        gtsam::Rot3 R = _relPose.rotation();

        std::string curEdgeInfo{
            "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " " + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z()) + " " + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w())};

        // pgEdgeSaveStream << curEdgeInfo << std::endl;
        edges_str.emplace_back(curEdgeInfo);
    }

    void writeRtkpose(const int _node_idx, const Eigen::Quaternion<double> &q_, const Eigen::Vector3d &t_)
    {
        std::string curVertexInfo{
            "RTKPOSE: " + std::to_string(_node_idx) + " " + std::to_string(t_(0)) + " " + std::to_string(t_(1)) + " " + std::to_string(t_(2)) + " " + std::to_string(q_.x()) + " " + std::to_string(q_.y()) + " " + std::to_string(q_.z()) + " " + std::to_string(q_.w())};
        rtkpose_str.emplace_back(curVertexInfo);
    }

    void splitPoseFileLine(std::string _str_line, vector<Eigen::Quaternion<double>> &_pointcloud_q, vector<Eigen::Vector3d> &_pointcloud_t)
    {
        std::stringstream ss(_str_line);
        std::vector<std::string> parsed_elms;
        std::string elm;
        char delimiter = ' ';
        Eigen::Matrix3d R;
        while (getline(ss, elm, delimiter))
        {
            parsed_elms.push_back(elm);
        }
        R << std::stod(parsed_elms.at(0)), std::stod(parsed_elms.at(1)), std::stod(parsed_elms.at(2)), std::stod(parsed_elms.at(4)), std::stod(parsed_elms.at(5)), std::stod(parsed_elms.at(6)), std::stod(parsed_elms.at(8)), std::stod(parsed_elms.at(9)), std::stod(parsed_elms.at(10));
        Eigen::Vector3d t(std::stod(parsed_elms.at(3)), std::stod(parsed_elms.at(7)), std::stod(parsed_elms.at(11)));
        _pointcloud_q.push_back(Eigen::Quaternion<double>(R));
        _pointcloud_t.push_back(t);
    }

    void get_number_of_lidar_topic()
    {
        if (update_map)
        {
            return;
        }
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string(lidar_topic));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        foreach (rosbag::MessageInstance const m, view)
        {
            lidar_topic_num++;
        }
        bag.close();
    }

    void mapping()
    {
        if (update_map)
        {
            return;
        }
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string(lidar_topic));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            std::string topic = m.getTopic();

            lidar_topic_count++;
            float percentage = (float)lidar_topic_count / lidar_topic_num;
            cout << "总共" << lidar_topic_num << "帧" << endl;
            cout << "\n当前建图进度：" << setprecision(4) << percentage * 100 << "%" << endl;

            sensor_msgs::PointCloud2::ConstPtr pcloud = m.instantiate<sensor_msgs::PointCloud2>();
            if (pcloud != NULL)
            {
                //根据差值获取当前雷达帧的时间对应的rtk位置
                Eigen::Quaternion<double> q_rtk_2_global(1, 0, 0, 0);
                Eigen::Vector3d t_rtk_2_global(0, 0, 0);
                odometry_time = pcloud->header.stamp.toSec();
                // printf("lidar time : %.5lf \n",odometry_time);
                bool interp = pose_manager.get_pose(odometry_time, q_rtk_2_global, t_rtk_2_global);
                //for the comaprasion of two rtk values
                q_rtk_gt = q_rtk_2_global;
                t_rtk_gt = t_rtk_2_global;

                if (!interp)
                    break;
                //根据标定外参获取激光雷达在全局雷达坐标系下的位置
                //全局雷达坐标系的原点取第一帧激光雷达所对应的rtk全局坐标系下的位姿
                // q_aa2bb 用途：把aa坐标系下的向量转到bb坐标系下 posebb = q_aa2bb*poseaa  (Q上bb下aa)
                // q_aa2bb = q_transformaa2bb 计算方法：把aa坐标系变换到bb坐标系使重合 的矩阵在bb坐标系下的表示
                // q_odom =  q_lidar2initlidar  =   q_lidar2rtk 乘q_rtk2global 乘q_golbal2initlidar
                // 由于向量旋转为左乘 故为 q_odom =  q_lidar2initlidar  =   q_lidar2rtk 乘q_rtk2global 乘q_golbal2initlidar = q_golbal2initlidar * q_rtk2global * q_lidar2rtk
                // 此处的global计算时采用相对于initrtk的变换(存rtk时补偿原点)
                q_lidar_rtk = q_imu2lidar * q_rtk_2_global * q_lidar2imu;
                t_lidar_rtk = q_imu2lidar.matrix() * (q_rtk_2_global.matrix() * t_lidar_2_imu + t_rtk_2_global) + t_imu_2_lidar;

                //keep point in range min ~ max
                pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(*pcloud, *tmp);
                pointcloud_in_range(min_distance, max_distance, tmp, pLaserCloud);

                //如果为第一帧雷达，则将rtk的值作为初值
                if (KeyFramePoses3D->points.empty() == true)
                {
                    q_odometry = q_lidar_rtk;
                    t_odometry = t_lidar_rtk;
                }
                else
                {
                    Eigen::Matrix4f trans(Eigen::Matrix4f::Identity());
                    // guess pose，根据新点云对应的时间的rtk和上一帧点云对应的时间的rtk，做差值并给上一帧的odometry作为先验
                    // input点云在这一帧，target点云是上一帧的子地图(全局坐标系下)，坐标系不同，init_guess是两个坐标系间的q t
                    // 点云的观测模型不是理想的，会受到视角影响，不过一般可以忽略；但可能影响ndt得分
                    // 此处t_odometry是上一帧的里程计位姿
                    Eigen::Vector3d t_guess = t_odometry + t_lidar_rtk - t_lidar_rtk_last;
                    Eigen::Quaterniond q_guess = q_lidar_rtk * q_lidar_rtk_last.conjugate() * q_odometry;
                    Eigen::Translation3d init_translation(t_guess(0), t_guess(1), t_guess(2));
                    Eigen::AngleAxisd init_rotation(q_guess);
                    Eigen::Matrix4d init_guess = (init_translation * init_rotation) * Eigen::Matrix4d::Identity();

                    TicToc time_ndt;
                    //downsample
                    downSizeFilter_25.setInputCloud(pLaserCloud);
                    downSizeFilter_25.filter(*pSourceCloud);

                    anh_ndt.setInputSource(pSourceCloud);
                    anh_ndt.align(init_guess.cast<float>());

                    //滤波后 雷达坐标系下点云 和 转到组合导航坐标系下子地图 作ndt 返回雷达坐标系在组合导航坐标系下位姿
                    trans = anh_ndt.getFinalTransformation();

                    Eigen::Vector3d dt(trans(0, 3), trans(1, 3), trans(2, 3));
                    Eigen::Matrix3d dR;
                    dR << trans(0, 0), trans(0, 1), trans(0, 2), trans(1, 0), trans(1, 1), trans(1, 2), trans(2, 0), trans(2, 1), trans(2, 2);

                    q_odometry = Eigen::Quaterniond(dR);
                    t_odometry = dt;

                    // printf("ndt time: %.2f ms, iteration : %d\n", time_ndt.toc(), anh_ndt.getFinalNumIteration());
                    dist += sqrt((t_lidar_rtk(0) - t_lidar_rtk_last(0)) * (t_lidar_rtk(0) - t_lidar_rtk_last(0)) + (t_lidar_rtk(1) - t_lidar_rtk_last(1)) * (t_lidar_rtk(1) - t_lidar_rtk_last(1)));
                }

                //更新上一帧的rtk位置
                q_lidar_rtk_last = q_lidar_rtk;
                t_lidar_rtk_last = t_lidar_rtk;
                //更新当前帧的位置
                currentPose.x = t_odometry(0);
                currentPose.y = t_odometry(1);
                currentPose.z = t_odometry(2);

                //TicToc time_save;
                bool newKeyFrame = saveKeyFrame();

                if (newKeyFrame)
                {
                    //TicToc time_LoopClosure;
                    detectLoopClosure();
                }

                //TicToc time_extract;
                extractSurroundingKeyFrames();

                printf("x : %.2lf, y : %.2lf, z : %.2lf, dist : %.2lf  \n", t_odometry(0), t_odometry(1), t_odometry(2), dist);
                if (newKeyFrame)
                    visulization();

                if (!ros::ok())
                    break;
            }
        }
        bag.close();
        printf("mapping finished !\n");
    }

    bool saveKeyFrame()
    {
        if (dist < map_dist)
            return false;

        dist -= map_dist;
        //如果为第一帧，构建因子图，插入先验信息
        if (KeyFramePoses3D->points.empty() == true)
        {
            gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3(q_odometry), Point3(t_odometry)), priorNoise));
            initialEstimate.insert(0, Pose3(Rot3(q_odometry), Point3(t_odometry)));
            writeVertexFromQT(0, q_odometry, t_odometry);
            loopClosure_time = odometry_time;
        }
        else
        {
            //每来一帧关键帧，则往后添加因子图，添加新的位姿节点和与上一个节点的边
            Eigen::Quaterniond q_last = pointcloud_q[pointcloud_q.size() - 1];
            Eigen::Vector3d t_last = pointcloud_t[pointcloud_t.size() - 1];

            gtsam::Pose3 poseFrom = Pose3(Rot3(q_last), Point3(t_last));
            gtsam::Pose3 poseTo = Pose3(Rot3(q_odometry), Point3(t_odometry));
            gtsam::Pose3 relPose = poseFrom.between(poseTo);

            gtSAMgraph.add(BetweenFactor<Pose3>(KeyFramePoses3D->points.size() - 1, KeyFramePoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(KeyFramePoses3D->points.size(), Pose3(Rot3(q_odometry), Point3(t_odometry)));
            writeVertex(KeyFramePoses3D->points.size(), poseTo);
            writeEdge({KeyFramePoses3D->size() - 1, KeyFramePoses3D->size()}, relPose); // giseop
        }

        //update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save result
        Pose3 latestEstimate;
        //更新获取优化后的q_sam，t_sam
        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        q_sam = latestEstimate.rotation().matrix();
        t_sam(0) = latestEstimate.translation().x();
        t_sam(1) = latestEstimate.translation().y();
        t_sam(2) = latestEstimate.translation().z();
        printf("      **** \n");
        printf("delta: %.2lf, %.2lf, %.2lf\n", t_sam(0) - t_odometry(0), t_sam(1) - t_odometry(1), t_sam(2) - t_odometry(2));
        //同时将 q_sam 和 t_sam 更新里程计的位姿
        q_odometry = q_sam;
        t_odometry = t_sam;
        //更新雷达的最终位姿
        pointcloud_q.push_back(q_odometry);
        pointcloud_t.push_back(t_odometry);
        pointcloud_time.push_back(odometry_time);
        //更新雷达的当前位姿
        currentPose.x = t_odometry(0);
        currentPose.y = t_odometry(1);
        currentPose.z = t_odometry(2);
        currentPose.intensity = pointcloud_q.size() - 1;
        //更新关键帧的位姿
        KeyFramePoses3D->points.push_back(currentPose);
        //更新rtk的位姿
        rtk_q.push_back(q_lidar_rtk);
        rtk_t.push_back(t_lidar_rtk);
        pcl::PointXYZI rtkcurrentPose;
        rtkcurrentPose.x = t_lidar_rtk(0);
        rtkcurrentPose.y = t_lidar_rtk(1);
        rtkcurrentPose.z = t_lidar_rtk(2);
        rtkKeyFramePoses3D->points.push_back(rtkcurrentPose);
        writeRtkpose(rtkKeyFramePoses3D->points.size() - 1, q_lidar_rtk, t_lidar_rtk);
        //存入关键帧的点云并构建scan context
        pcl::PointCloud<pcl::PointXYZI>::Ptr thisKeyFrame(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*pLaserCloud, *thisKeyFrame);
        CloudKeyFrames.push_back(thisKeyFrame);
        scManager.makeAndSaveScancontextAndKeys(*thisKeyFrame);
        const auto &curr_scd = scManager.getConstRefRecentSCD();
        std::string curr_scd_node_idx = padZeros(scManager.polarcontexts_.size() - 1);
        saveSCD(sc_path + "/" + curr_scd_node_idx + ".scd", curr_scd);

        pcl::PointCloud<pcl::PointXYZI>::Ptr thisKeyFrameDS(new pcl::PointCloud<pcl::PointXYZI>());
        downSizeFilter_50.setInputCloud(thisKeyFrame);
        downSizeFilter_50.filter(*thisKeyFrameDS);
        thisKeyFrameDS->height = 1;
        thisKeyFrameDS->width = thisKeyFrameDS->points.size();
        std::string key_frames_pcd = key_frames_path + "/" + std::to_string(CloudKeyFrames.size()) + ".pcd";
        pcl::io::savePCDFileASCII(key_frames_pcd, *thisKeyFrameDS);
        std::string dense_key_frames_pcd = dense_key_frames_path + "/" + std::to_string(CloudKeyFrames.size()) + ".pcd";
        pcl::io::savePCDFileASCII(dense_key_frames_pcd, *thisKeyFrame);

        pcl::io::savePCDFileBinary(scans_path + "/" + curr_scd_node_idx + ".pcd", *thisKeyFrameDS);
        pgTimeSaveStream << odometry_time << std::endl;

        printf("save keyframe pcd at : %s \n", key_frames_pcd.c_str());
        printf("save keyframe %ld !!! \n", pointcloud_q.size() - 1);
        printf("t_odometry: %.2lf, %.2lf, %.2lf\n", t_odometry(0), t_odometry(1), t_odometry(2));
        printf("t_lidar_rtk: %.2lf, %.2lf, %.2lf\n", t_lidar_rtk(0), t_lidar_rtk(1), t_lidar_rtk(2));

        return true;
    }

    void detectLoopClosure()
    {
        pRSTargetCloud->clear();
        detected_history_keyframe_id = -1;
        //将关键帧对应的rtk的位姿构建一个kd-tree，然后根据当前帧对应的rtk位置寻找最近的几个点，
        //再寻找这些点里时间最早的那个点，将id赋值给RSclosestHistoryFrameID
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurroundingKeyPoses->setInputCloud(rtkKeyFramePoses3D);
        pcl::PointXYZI searchPose;
        searchPose.x = t_lidar_rtk(0);
        searchPose.y = t_lidar_rtk(1);
        searchPose.z = t_lidar_rtk(2);
        kdtreeSurroundingKeyPoses->radiusSearch(searchPose, surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);

        int curMinID = 1000000;
        for (int i = 0; i < pointSearchInd.size(); i++)
        {
            int id = pointSearchInd[i];
            // TODO
            if (abs(pointcloud_time[id] - odometry_time) <= 15)
                continue;

            if (id < curMinID)
            {
                curMinID = id;
                detected_history_keyframe_id = curMinID;
            }
        }

        if (detected_history_keyframe_id == -1)
            return;

        if (abs(odometry_time - loopClosure_time) <= 30)
        {
            printf("loop time check failed: %.3lf - %.3lf \n", odometry_time, loopClosure_time);
            return;
        }

        int current_keyframe_id = pointcloud_q.size() - 1;
        printf("detect loop closure id : %d-%d !!! \n", current_keyframe_id, detected_history_keyframe_id);
        //根据最近的回环的id找到其前后五帧，构建回环匹配的局部地图pRSTargetCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
        for (int j = -recentKeyFramesNum / 2; j <= recentKeyFramesNum / 2; j++)
        {
            int id = detected_history_keyframe_id + j;
            if (id < 0 || id > KeyFramePoses3D->points.size() - 1)
                continue;
            printf("loopClosure: add id %d pointcloud to RSTargetCloud \n", id);
            *tmp += *transformPointCloud(pointcloud_q[id], pointcloud_t[id], CloudKeyFrames[id]);
        }

        downSizeFilter_50.setInputCloud(tmp);
        downSizeFilter_50.filter(*pRSTargetCloud);

        //将当前关键帧对应的点云根据最新的qt转换坐标得到pRSSourceCloud和局部地图匹配
        pcl::PointCloud<pcl::PointXYZI>::Ptr pRSSourceCloud(new pcl::PointCloud<pcl::PointXYZI>());
        pRSSourceCloud = transformPointCloud(pointcloud_q[current_keyframe_id], pointcloud_t[current_keyframe_id], CloudKeyFrames[current_keyframe_id]);

        // debug
        // save candidate pointcloud to pcd
        // string source_pcd = map_path + "/debug_map/" + to_string(close_index) + "_" + to_string(current_keyframe_id) +"_source.pcd";
        // string target_pcd = map_path + "/debug_map/" + to_string(close_index) + "_" + to_string(detected_history_keyframe_id) +"_target.pcd";
        // pcl::io::savePCDFileASCII(source_pcd, *pRSSourceCloud);
        // pcl::io::savePCDFileASCII(target_pcd, *pRSTargetCloud);
        // close_index++;

        cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
        ndt.setResolution(1);
        ndt.setMaximumIterations(500);
        ndt.setStepSize(0.1);
        ndt.setTransformationEpsilon(0.01);
        ndt.setInputSource(pRSSourceCloud);
        ndt.setInputTarget(pRSTargetCloud);

        Eigen::Matrix4f trans(Eigen::Matrix4f::Identity());
        // 此处都转到全局坐标系下，q和t应该都是单位阵和(0,0,0) 此处初值没意义 可拿有回环场景测试匹配率
        Eigen::Quaterniond q_guess(1, 0, 0, 0);
        // Eigen::Vector3d t_guess = (pointcloud_t[detected_history_keyframe_id] - pointcloud_t[current_keyframe_id]) - (rtk_t[detected_history_keyframe_id] - rtk_t[current_keyframe_id]);
        Eigen::Vector3d t_guess(0, 0, 0);
        printf("q_guess: %.2lf, %.2lf, %.2lf, %.2lf\n", q_guess.x(), q_guess.y(), q_guess.z(), q_guess.w());
        printf("t_guess: %.2lf, %.2lf, %.2lf\n", t_guess(0), t_guess(1), t_guess(2));
        Eigen::Translation3d init_translation(t_guess(0), t_guess(1), t_guess(2));
        Eigen::AngleAxisd init_rotation(q_guess);
        Eigen::Matrix4d init_guess = (init_translation * init_rotation) * Eigen::Matrix4d::Identity();
        ndt.align(init_guess.cast<float>());

        trans = ndt.getFinalTransformation();
        printf("%.4f, %.4f, %.4f, %.4f \n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
        printf("%.4f, %.4f, %.4f, %.4f \n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
        printf("%.4f, %.4f, %.4f, %.4f \n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        printf("%.4f, %.4f, %.4f, %.4f \n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));

        Eigen::Vector3d dt(trans(0, 3), trans(1, 3), trans(2, 3));
        Eigen::Matrix3d dR;
        dR << trans(0, 0), trans(0, 1), trans(0, 2), trans(1, 0), trans(1, 1), trans(1, 2), trans(2, 0), trans(2, 1), trans(2, 2);
        //获取回环检测的qt
        Eigen::Quaterniond q_lc = Eigen::Quaterniond(dR);
        Eigen::Vector3d t_lc = dt;

        //ndt result
        double fitness_score = ndt.getFitnessScore();
        printf("loop closure ndt result : \n");
        printf("hasConverged : %d\n", ndt.hasConverged());
        printf("iteration : %d\n", ndt.getFinalNumIteration());
        printf("fitness_score : %f\n", fitness_score);
        printf("trans_probability :%f\n", ndt.getTransformationProbability());

        //score check
        if (fitness_score > 1.2)
        {
            printf("bad match !!! \n");

            return;
        }

        loopClosure_time = odometry_time;

        float noiseScore = 0.5; //TODO
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        constraintNoise = noiseModel::Diagonal::Variances(Vector6);
        robustNoiseModel = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
            gtsam::noiseModel::Diagonal::Variances(Vector6));
        //构建回环历史帧的位姿节点
        Eigen::Quaterniond q_target = pointcloud_q[detected_history_keyframe_id];
        Eigen::Vector3d t_target = pointcloud_t[detected_history_keyframe_id];
        //将当前帧的位姿根据相对回环构建节点
        Eigen::Quaterniond q_check = q_lc * pointcloud_q[current_keyframe_id];
        Eigen::Vector3d t_check = q_lc * pointcloud_t[current_keyframe_id] + t_lc;

        printf("before : %.2lf, %.2lf, %.2lf \n", t_odometry(0), t_odometry(1), t_odometry(2));
        printf("check  : %.2lf, %.2lf, %.2lf \n", t_check(0), t_check(1), t_check(2));

        gtsam::Pose3 poseFrom = Pose3(Rot3(q_check), Point3(t_check));
        gtsam::Pose3 poseTo = Pose3(Rot3(q_target), Point3(t_target));
        //构建回环顶点约束
        gtSAMgraph.add(BetweenFactor<Pose3>(current_keyframe_id, detected_history_keyframe_id, poseFrom.between(poseTo), robustNoiseModel));
        isam->update(gtSAMgraph);
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isamCurrentEstimate = isam->calculateEstimate();
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        writeEdge({current_keyframe_id, detected_history_keyframe_id}, poseFrom.between(poseTo)); // write loop edge into .g2o

        //update keyframe match_map
        //keyframe
        int numPoses = isamCurrentEstimate.size();

        for (int i = 0; i < numPoses; i++)
        {
            double x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            double y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            double z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            // printf("%d : %.2lf, %.2lf, %.2lf : %.2lf, %.2lf, %.2lf\n", i, pointcloud_t[i](0), pointcloud_t[i](1), pointcloud_t[i](2), x, y, z);
            //将回环后的所有新位姿重新赋值给pointcloud_q,t
            pointcloud_q[i] = isamCurrentEstimate.at<Pose3>(i).rotation().matrix();
            pointcloud_t[i](0) = x;
            pointcloud_t[i](1) = y;
            pointcloud_t[i](2) = z;
            //更新keyframe的所有位姿
            KeyFramePoses3D->points[i].x = x;
            KeyFramePoses3D->points[i].y = y;
            KeyFramePoses3D->points[i].z = z;

            if (i < numPoses - 1)
            {
                path.poses[i].pose.orientation.x = pointcloud_q[i].x();
                path.poses[i].pose.orientation.y = pointcloud_q[i].y();
                path.poses[i].pose.orientation.z = pointcloud_q[i].z();
                path.poses[i].pose.orientation.w = pointcloud_q[i].w();
                path.poses[i].pose.position.x = x;
                path.poses[i].pose.position.y = y;
                path.poses[i].pose.position.z = z;
            }
        }

        //更新当前帧的位姿点
        currentPose.x = isamCurrentEstimate.at<Pose3>(numPoses - 1).translation().x();
        currentPose.y = isamCurrentEstimate.at<Pose3>(numPoses - 1).translation().y();
        currentPose.z = isamCurrentEstimate.at<Pose3>(numPoses - 1).translation().z();
        Eigen::Quaterniond q_tmp = pointcloud_q[current_keyframe_id];
        Eigen::Vector3d t_tmp = pointcloud_t[current_keyframe_id];
        //回环后记得更新最新的雷达里程计
        q_odometry = q_tmp;
        t_odometry = t_tmp;
        printf("sam : %.2lf, %.2lf, %.2lf\n", currentPose.x, currentPose.y, currentPose.z);

        //match_map
        recentCloudKeyFrames.clear();
        for (int i = 0; i < recentKeyFramesNum; i++)
        {
            int id = numPoses - 1 - i;
            if (id >= 0)
                recentCloudKeyFrames.push_front(transformPointCloud(pointcloud_q[id], pointcloud_t[id], CloudKeyFrames[id]));
        }
        latestFrameID = KeyFramePoses3D->points.size() - 1;

        //printf("generate new ndt target pointcloud !!! \n");
        //generate new ndt target pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZI>());
        for (int i = 0; i < recentCloudKeyFrames.size(); i++)
            *tmp2 += *recentCloudKeyFrames[i];

        downSizeFilter_50.setInputCloud(tmp2);
        downSizeFilter_50.filter(*pTargetCloud);
        anh_ndt.setInputTarget(pTargetCloud);
    }

    void extractSurroundingKeyFrames()
    {
        bool new_target_cloud = false;

        if (KeyFramePoses3D->points.empty() == true)
        {
            return;
        }

        if (recentCloudKeyFrames.size() < recentKeyFramesNum)
        {
            if (latestFrameID != KeyFramePoses3D->points.size() - 1)
            {
                latestFrameID = KeyFramePoses3D->points.size() - 1;
                recentCloudKeyFrames.push_back(transformPointCloud(pointcloud_q[latestFrameID], pointcloud_t[latestFrameID], CloudKeyFrames[latestFrameID]));
                new_target_cloud = true;
            }
        }
        else
        {
            if (latestFrameID != KeyFramePoses3D->points.size() - 1)
            {
                recentCloudKeyFrames.pop_front();
                latestFrameID = KeyFramePoses3D->points.size() - 1;
                recentCloudKeyFrames.push_back(transformPointCloud(pointcloud_q[latestFrameID], pointcloud_t[latestFrameID], CloudKeyFrames[latestFrameID]));
                new_target_cloud = true;
            }
        }

        if (!new_target_cloud)
            return;

        //printf("generate new ndt target pointcloud !!! \n");
        //generate new ndt target pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
        for (int i = 0; i < recentCloudKeyFrames.size(); i++)
            *tmp += *recentCloudKeyFrames[i];

        downSizeFilter_50.setInputCloud(tmp);
        downSizeFilter_50.filter(*pTargetCloud);
        anh_ndt.setInputTarget(pTargetCloud);
    }

    void visulization()
    {
        geometry_msgs::PoseStamped odom;
        odom.pose.orientation.x = q_odometry.x();
        odom.pose.orientation.y = q_odometry.y();
        odom.pose.orientation.z = q_odometry.z();
        odom.pose.orientation.w = q_odometry.w();
        odom.pose.position.x = t_odometry(0);
        odom.pose.position.y = t_odometry(1);
        odom.pose.position.z = t_odometry(2);
        odom.header.frame_id = "/map";
        odom.header.stamp = ros::Time::now();
        pubOdom.publish(odom);

        path.poses.push_back(odom);
        path.header.frame_id = "/map";
        path.header.stamp = odom.header.stamp;
        pubPath.publish(path);

        sensor_msgs::PointCloud2 TargetCloud2;
        pcl::toROSMsg(*pTargetCloud, TargetCloud2);
        TargetCloud2.header.stamp = odom.header.stamp;
        TargetCloud2.header.frame_id = "/map";
        pubMap.publish(TargetCloud2);

        geometry_msgs::PoseStamped rtk_odom;
        rtk_odom.pose.position.x = t_lidar_rtk(0);
        rtk_odom.pose.position.y = t_lidar_rtk(1);
        rtk_odom.pose.position.z = t_lidar_rtk(2);
        rtk_odom.pose.orientation.x = q_lidar_rtk.x();
        rtk_odom.pose.orientation.y = q_lidar_rtk.y();
        rtk_odom.pose.orientation.z = q_lidar_rtk.z();
        rtk_odom.pose.orientation.w = q_lidar_rtk.w();
        rtk_odom.header.frame_id = "/map";
        rtk_odom.header.stamp = ros::Time::now();
        pubrtk.publish(rtk_odom);

        rtk_path.poses.push_back(rtk_odom);
        rtk_path.header.frame_id = "/map";
        rtk_path.header.stamp = rtk_odom.header.stamp;
        pubRtkPath.publish(rtk_path);

        geometry_msgs::PoseStamped rtk_gt;
        rtk_gt.pose.position.x = t_rtk_gt(0);
        rtk_gt.pose.position.y = t_rtk_gt(1);
        rtk_gt.pose.position.z = t_rtk_gt(2);
        rtk_gt.pose.orientation.x = q_rtk_gt.x();
        rtk_gt.pose.orientation.y = q_rtk_gt.y();
        rtk_gt.pose.orientation.z = q_rtk_gt.z();
        rtk_gt.pose.orientation.w = q_rtk_gt.w();
        rtk_gt.header.frame_id = "/map";
        rtk_gt.header.stamp = ros::Time::now();

        rtk_gt_path.poses.push_back(rtk_gt);
        rtk_gt_path.header.frame_id = "/map";
        rtk_gt_path.header.stamp = rtk_gt.header.stamp;
        pubRtkGt.publish(rtk_gt_path);
    }

    //分离地图合并模块
    // 或者采用触发式？
    //从txt读取R，t 从Scans读取关键帧
    void saveMap()
    {
        printf("wait a second ... \n");
        printf("final map visulization ... \n");

        pcl::PointCloud<pcl::PointXYZI>::Ptr pSaveMap(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pSaveMapDS(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pDenseMap(new pcl::PointCloud<pcl::PointXYZI>());

        //从更新后的位姿读取q和t;
        if (update_map)
        {
            pointcloud_q.clear();
            pointcloud_t.clear();
            cout << "optimized_pose_path: " << optimized_pose_path << endl;
            std::ifstream posefile_handle(optimized_pose_path);
            std::string strOneLine;
            while (getline(posefile_handle, strOneLine))
            {
                // 在第一次建图坐标系下的q t
                splitPoseFileLine(strOneLine, pointcloud_q, pointcloud_t);
            }
            cout << "size of pointcloud_q:  " << pointcloud_q.size() << endl;
            cout << "总共" << pointcloud_q.size() << "帧" << endl;
            // load pointcloud
            for (int i = 1; i <= pointcloud_q.size(); i++)
            {
                string init_cloud_path = dense_key_frames_path + "/" + std::to_string(i) + ".pcd";
                pcl::PointCloud<pcl::PointXYZI>::Ptr init_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(init_cloud_path, *init_cloud) == -1)
                {
                    PCL_ERROR("Could not read file :  %s \n", init_cloud_path.c_str());
                    return;
                }
                CloudKeyFrames.push_back(init_cloud);
                float percentage = (float)i / pointcloud_q.size();
                cout << "\n当前更新进度：" << setprecision(4) << percentage * 100 << "%" << endl;
            }
            // proved unused
            // double origin_yaw_in_central_sess;
            // Eigen::Vector3d eulerAngle = pointcloud_q[0].matrix().eulerAngles(2, 1, 0);
            // origin_yaw_in_central_sess = eulerAngle(0) / M_PI * 180;
            // std::cout << "[revised]: The heading of origin is : " << origin_heading << " - " << origin_yaw_in_central_sess << std::endl;
            // origin_heading = origin_heading - origin_yaw_in_central_sess;
        }
        for (int i = 0; i < CloudKeyFrames.size(); i++)
        {
            if (i * save_map_dist > CloudKeyFrames.size())
            {
                break;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            cloud_after_filtered = PointCloudFilter(CloudKeyFrames[i * save_map_dist]);
            *pSaveMap += *transformPointCloud(pointcloud_q[i * save_map_dist], pointcloud_t[i * save_map_dist], cloud_after_filtered);
            *pDenseMap += *transformPointCloud(pointcloud_q[i * save_map_dist], pointcloud_t[i * save_map_dist], CloudKeyFrames[i * save_map_dist]);
        }
        downSizeFilter_25.setInputCloud(pSaveMap);
        downSizeFilter_25.filter(*pSaveMapDS);

        pSaveMapDS->height = 1;
        pSaveMapDS->width = pSaveMapDS->points.size();

        pDenseMap->height = 1;
        pDenseMap->width = pDenseMap->points.size();

        sensor_msgs::PointCloud2 FinalMap2;
        pcl::toROSMsg(*pSaveMapDS, FinalMap2);
        FinalMap2.header.stamp = ros::Time().now();
        FinalMap2.header.frame_id = "/map";
        pubMap.publish(FinalMap2);

        if (!update_map)
        {
            printf("saving TUM format... \n");
            saveTUMformat();
        }

        if (!save_map)
        {
            printf("88 \n");
            return;
        }

        printf("saving map ... \n");
        pcl::io::savePCDFileASCII(original_map_pcd, *pSaveMapDS);
        printf("save map pcd at %s \n", original_map_pcd.c_str());
        pcl::io::savePCDFileASCII(dense_map_pcd, *pDenseMap);

        // save submap
        // map<string, pcl::PointCloud<pcl::PointXYZI>::Ptr> map_name_cloud;
        // map<string, pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator it;
        // for (uint32_t i = 0; i < pSaveMapDS->points.size(); i++)
        // {
        //     double px = pSaveMapDS->points[i].x;
        //     double py = pSaveMapDS->points[i].y;

        //     int x = floor(px / submap_size);
        //     int y = floor(py / submap_size);

        //     string str = std::to_string(x) + "_" + std::to_string(y);

        //     it = map_name_cloud.find(str);

        //     //如果该部分子地图之前出现过，直接存入
        //     if (it != map_name_cloud.end())
        //     {
        //         //往对应子地图中存入点云
        //         pcl::PointCloud<pcl::PointXYZI>::Ptr tmp = it->second;
        //         tmp->points.push_back(pSaveMapDS->points[i]);
        //     }

        //     else
        //     {
        //         pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
        //         tmp->points.push_back(pSaveMapDS->points[i]);
        //         map_name_cloud.insert(std::pair<string, pcl::PointCloud<pcl::PointXYZI>::Ptr>(str, tmp));
        //     }
        // }
        // to pcd file
        // for (it = map_name_cloud.begin(); it != map_name_cloud.end(); it++)
        // {
        //     string str = it->first;
        //     string submap_pcd = submap_path + "/" + str + ".pcd";

        //     pcl::PointCloud<pcl::PointXYZI>::Ptr tmp = it->second;
        //     tmp->height = 1;
        //     tmp->width = tmp->points.size();

        //     pcl::io::savePCDFileASCII(submap_pcd, *tmp);
        //     printf("save submap pcd at : %s \n", submap_pcd.c_str());
        // }

        /**
         * @name:
         * @descrip: for erasor use , pub node topic 
         * @param {int} i
         * @return {*}
         */

        // for (int i = 0; i < pointcloud_q.size(); i++)
        // {
        //     pcl::PointCloud<pcl::PointXYZI>::Ptr KeyFrameDS(new pcl::PointCloud<pcl::PointXYZI>());
        //     downSizeFilter_25.setInputCloud(CloudKeyFrames[i]);
        //     downSizeFilter_25.filter(*KeyFrameDS);
        //     sensor_msgs::PointCloud2 node_lidar;
        //     pcl::toROSMsg(*KeyFrameDS, node_lidar);
        //     geometry_msgs::Pose node_odom;
        //     node_odom.orientation.x = pointcloud_q[i].x();
        //     node_odom.orientation.y = pointcloud_q[i].y();
        //     node_odom.orientation.z = pointcloud_q[i].z();
        //     node_odom.orientation.w = pointcloud_q[i].w();
        //     node_odom.position.x = pointcloud_t[i].x();
        //     node_odom.position.y = pointcloud_t[i].y();
        //     node_odom.position.z = pointcloud_t[i].z();
        //     rtk_mapping::node publishednode;
        //     publishednode.header.stamp = ros::Time::now();
        //     publishednode.header.seq = i;
        //     publishednode.header.frame_id = "/map";
        //     publishednode.lidar = node_lidar;
        //     publishednode.odom = node_odom;
        //     //为了找到是哪一时刻对应的id
        //     cout << "The current published node id is : " << i << " The current stamp is : " << publishednode.header.stamp << endl;
        //     pubNode.publish(publishednode);
        // }

        /* -------------------------------------------------------------------------- */
        /*                transform the pc coordinate towards the north               */
        /* -------------------------------------------------------------------------- */
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(original_map_pcd, *pcd_cloud) == -1)
        {
            PCL_ERROR("Could not read file :  %s \n", original_map_pcd.c_str());
            return;
        }
        double torad_ = M_PI / 180;
        Eigen::Matrix3d R_ROTATION_HEADING;
        Eigen::Quaterniond q_rotation_heading;
        Eigen::Vector3d t_rotation_heading(0.0, 0.0, 0.0);
        R_ROTATION_HEADING = Eigen::AngleAxisd(-(origin_heading * torad_), Eigen::Vector3d::UnitZ());
        std::cout << "origin_heading is : " << origin_heading << std::endl;
        q_rotation_heading = R_ROTATION_HEADING;
        *cloud_after_rotation = *transformPointCloud(q_rotation_heading, t_rotation_heading, pcd_cloud);
        cloud_after_rotation->height = 1;
        cloud_after_rotation->width = cloud_after_rotation->points.size();
        pcl::io::savePCDFileASCII(final_map_path, *cloud_after_rotation);
        if (update_map)
        {
            return;
        }

        // save pose graph (runs when programe is closing)
        printf("pcd map saved ~~~ \n");
        cout << "****************************************************" << endl;
        cout << "Saving the posegraph ..." << endl;

        for (auto &_line : vertices_str)
            pgSaveStream << _line << std::endl;
        for (auto &_line : edges_str)
            pgSaveStream << _line << std::endl;
        pgSaveStream.close();
        cout << vertices_str.size() << endl;
        for (auto &_line : rtkpose_str)
            rtkSaveStream << _line << std::endl;
        rtkSaveStream.close();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());

        //use the ransac to remove the plane
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.3);
        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            cout << "error! Could not found any inliers!" << endl;
            return cloud_out;
        }
        pcl::ExtractIndices<pcl::PointXYZI> extractor;
        extractor.setInputCloud(cloud_in);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*cloud_filtered);

        //use the radiusoutler filter to remove some outliers
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusoutlier;
        radiusoutlier.setInputCloud(cloud_filtered);
        radiusoutlier.setRadiusSearch(filter_radius);
        radiusoutlier.setMinNeighborsInRadius(filter_count);
        radiusoutlier.filter(*cloud_tmp);

        cloud_out = cloud_tmp;

        return cloud_out;
    }

    void saveTUMformat()
    {
        char end1 = 0x0d; // "/n"
        char end2 = 0x0a;

        // rtk odometry
        FILE *fp = NULL;

        string rtk_tum_file = map_path + "/rtk.txt";
        fp = fopen(rtk_tum_file.c_str(), "w+");

        if (fp == NULL)
        {
            printf("fail to open file (rtk odometry file) ! \n");
            exit(1);
        }
        else
            printf("TUM : write rtk data to %s \n", rtk_tum_file.c_str());

        for (int i = 0; i < pose_manager.map_q_.size(); i++)
        {
            Eigen::Quaterniond q = pose_manager.map_q_[i];
            Eigen::Vector3d t = pose_manager.map_t_[i];
            double time = pose_manager.map_time_[i];
            fprintf(fp, "%.3lf %.3lf %.3lf %.3lf %.5lf %.5lf %.5lf %.5lf%c",
                    time, t(0), t(1), t(2),
                    q.x(), q.y(), q.z(), q.w(), end2);
        }

        fclose(fp);

        // lidar odometry
        string lidar_tum_file = map_path + "/lidar.txt";
        fp = fopen(lidar_tum_file.c_str(), "w+");

        if (fp == NULL)
        {
            printf("fail to open file (lidar odometry file) ! \n");
            exit(1);
        }
        else
            printf("TUM : write lidar data to %s \n", lidar_tum_file.c_str());

        for (int i = 0; i < pointcloud_q.size(); i++)
        {
            Eigen::Quaterniond q = pointcloud_q[i];
            Eigen::Vector3d t = pointcloud_t[i];
            double time = pointcloud_time[i];
            fprintf(fp, "%.3lf %.3lf %.3lf %.3lf %.5lf %.5lf %.5lf %.5lf%c",
                    time, t(0), t(1), t(2),
                    q.x(), q.y(), q.z(), q.w(), end2);
        }
        fclose(fp);

        string sc_txt = map_path + "/sc.txt";
        fp = fopen(sc_txt.c_str(), "w+");

        if (fp == NULL)
        {
            printf("fail to open file (sc file)!\n");
            exit(1);
        }
        else
            printf("sc:write sc data to %s \n", sc_txt.c_str());

        for (int i = 0; i < pointcloud_q.size(); i++)
        {
            Eigen::Quaterniond q = pointcloud_q[i];
            Eigen::Vector3d t = pointcloud_t[i];
            fprintf(fp, "%.3lf %.3lf %.3lf %.5lf %.5lf %.5lf %.5lf%c",
                    t(0), t(1), t(2),
                    q.x(), q.y(), q.z(), q.w(), end2);
        }
        fclose(fp);

        std::ofstream os(map_path + "/PoseGraphTest.dot");
        gtSAMgraph.saveGraph(os, isamCurrentEstimate);
    }

private:
    //debug
    int close_index;
    //ros
    ros::NodeHandle nh_;

    ros::Publisher pubMap;
    ros::Publisher pubOdom;
    ros::Publisher pubPath;
    ros::Publisher pubNode;
    ros::Publisher pubrtk;
    ros::Publisher pubRtkPath;
    ros::Publisher pubOccupancyGridMap;
    ros::Publisher pubRtkGt;

    nav_msgs::Path path;
    nav_msgs::Path rtk_path;
    nav_msgs::Path rtk_gt_path;

    //params from launch
    string map_path;
    string bag_path;
    bool save_map;
    bool update_map;
    string sc_path;
    string key_frames_path;
    string dense_key_frames_path;
    string lidar_topic;
    string imu_topic;
    string update_map_path;

    /* -------------------------------------------------------------------------- */
    /*                          occupancy grid map param                          */
    /* -------------------------------------------------------------------------- */
    // string map_pcd_path;
    double min_distance;
    double max_distance;
    int occupancy_grid_map_width;
    int occupancy_grid_map_height;
    int occupancy_grid_map_length;
    vector<uint8_t> occupancy_grid_map_data;

    double map_dist; // keyframe dist
    double dist;

    int submap_size;
    int save_map_dist; // add a keyframe to map in a set of save_map_dist keyframes

    //rtk data manager
    rtk_pose pose_manager;

    //calibration
    Eigen::Matrix3d R_imu2lidar;
    Eigen::Quaterniond q_imu2lidar;
    Eigen::Quaterniond q_lidar2imu;
    Eigen::Vector3d t_lidar_2_imu;
    Eigen::Vector3d t_imu_2_lidar;

    //pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr pLaserCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pLaserCloudDS;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pMapCloudDS;
    //map for occupancy grid
    pcl::PointCloud<pcl::PointXYZI>::Ptr pMapCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pTargetCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pSourceCloud;
    //cloud after transform in occupancy grid map creation
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_rotation;

    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CloudKeyFrames;
    deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> recentCloudKeyFrames;
    double recentKeyFramesNum;
    int latestFrameID;

    //keyframe pose
    pcl::PointCloud<pcl::PointXYZI>::Ptr KeyFramePoses3D;
    pcl::PointXYZI currentPose;
    double odometry_time;
    //最终的雷达点云位姿
    vector<Eigen::Quaternion<double>> pointcloud_q;
    vector<Eigen::Vector3d> pointcloud_t;
    vector<double> pointcloud_time;
    //keyframe对应的rtk位
    pcl::PointCloud<pcl::PointXYZI>::Ptr rtkKeyFramePoses3D;
    vector<Eigen::Quaternion<double>> rtk_q;
    vector<Eigen::Vector3d> rtk_t;

    //pose，雷达的rtk值
    Eigen::Quaternion<double> q_lidar_rtk_last;
    Eigen::Vector3d t_lidar_rtk_last;
    Eigen::Quaternion<double> q_lidar_rtk;
    Eigen::Vector3d t_lidar_rtk;
    //里程计位姿
    Eigen::Quaternion<double> q_odometry;
    Eigen::Vector3d t_odometry;
    //优化后的位姿
    Eigen::Quaternion<double> q_sam;
    Eigen::Vector3d t_sam;

    //ground truth rtk
    Eigen::Quaternion<double> q_rtk_gt;
    Eigen::Vector3d t_rtk_gt;

    //VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_50;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_25;

    //ndt
    cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;

    //pass_through_filter
    pcl::PassThrough<pcl::PointXYZI> passthrough;

    //mutex
    std::mutex mtx;

    //scan context
    SCManager scManager;

    std::fstream pgTimeSaveStream; // pg: pose-graph
    std::fstream pgSaveStream;
    std::fstream rtkSaveStream;
    std::vector<std::string> edges_str;
    std::vector<std::string> vertices_str;
    std::vector<std::string> rtkpose_str;

    //loop closure
    double loopClosure_time;
    int detected_history_keyframe_id;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pRSTargetCloud;

    //Kdtree
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurroundingKeyPoses;
    double surroundingKeyframeSearchRadius;

    //gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    noiseModel::Diagonal::shared_ptr constraintNoise;
    noiseModel::Base::shared_ptr robustNoiseModel;

    //global map's origin information
    double origin_heading;
    bool get_map_origin_informaiton;
    double new_origin_x;
    double new_origin_y;
    //filter param
    double filter_radius;
    int filter_count;
    //processing monitor param
    int lidar_topic_num;
    int lidar_topic_count;
    // string saved_json_path;
    string final_map_path;
    string optimized_pose_path;
    string former_bag_path;
    string original_map_pcd;
    string dense_map_pcd;
    string submap_path;
    string scans_path;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map");
    ros::NodeHandle nh;

    MapOptimization MO(nh);
    //first traverse the rtk data and get the whole pose vector
    MO.getRtkPose();
    //then start mapping,read lidar data and get the nearest rtk data to map
    MO.get_number_of_lidar_topic();
    MO.mapping();
    // //when finish mapping , save map and get occupancy grid map format
    MO.saveMap();

    return 0;
}