#include "map_manager.h"
#include "tictoc.h"

map_manager::map_manager(string pcd_path, string submap_path, string sc_path)
{
    pcd_path_ = pcd_path;
    submap_path_ = submap_path;
    sc_path_ = sc_path;

    map_centerx_ = -10000;
    map_centery_ = -10000;
}

void map_manager::load_map()
{
    // load map and downsize
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    map_ = raw_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *map_) == -1)
    {
        printf("can't read open pcd \n");
        exit(-1);
    }
    else
    {
        printf("open file : %s \n", pcd_path_.c_str());
    }

    printf("map loaded success and point nums : %d \n", map_->points.size());

    // load sc pose
    string sc_txt = sc_path_ + "/sc.txt";
    FILE *sc_read_fp = NULL;
    sc_read_fp = fopen(sc_txt.c_str(), "r");

    if (sc_read_fp == NULL)
    {
        printf("fail to open sc txt !!!\n");
        exit(1);
    }
    else
    {
        vector<double> tmp(7, 0);
        while (!feof(sc_read_fp))
        {
            fscanf(sc_read_fp, "%lf %lf %lf %lf %lf %lf %lf", &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5], &tmp[6]);
            Eigen::Vector3d t(tmp[0], tmp[1], tmp[2]);
            Eigen::Quaterniond q(tmp[6], tmp[3], tmp[4], tmp[5]);
            sc_t_.push_back(t);
            sc_q_.push_back(q);
        }
        sc_t_.pop_back();
        sc_q_.pop_back();
    }

    // get sc
    for (uint32_t i = 0; i < sc_t_.size(); i++)
    {
        int x = round(sc_t_[i](0) * 100);
        int y = round(sc_t_[i](1) * 100);
        string pcd_file = sc_path_ + "/pcd/" + std::to_string(x) + "_" + std::to_string(y) + ".pcd";

        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *tmp) == -1)
        {
            printf("can't read open sc pcd \n");
            exit(-1);
        }
        else
        {
            printf("%.3lf, %.3lf open sc file : %s \n", sc_t_[i](0), sc_t_[i](1), pcd_file.c_str());
            sc_.makeAndSaveScancontextAndKeys(*tmp);
        }
    }
}

void map_manager::init_location(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                                Eigen::Quaterniond &q_out, Eigen::Vector3d &t_out)
{
    std::pair<int, float> result{-1, 0};
    result = sc_.relocalize(*cloud_in);
    printf("match result : %d, %f \n", result.first, result.second / M_PI * 180);

    if (result.first == -1)
    {
        printf("init location failed !!! \n");
        return;
    }

    int x = round(sc_t_[result.first](0) * 100);
    int y = round(sc_t_[result.first](1) * 100);

    Eigen::Quaterniond q_match = sc_q_[result.first];
    Eigen::Vector3d t_match = sc_t_[result.first];

    string pcd_file = sc_path_ + "/pcd/" + std::to_string(x) + "_" + std::to_string(y) + ".pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_match(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_match) == -1)
    {
        printf("init_location can't read open pcd \n");
        exit(-1);
    }

    cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;
    anh_ndt.setResolution(1.0);
    anh_ndt.setMaximumIterations(30);
    anh_ndt.setStepSize(0.1);
    anh_ndt.setTransformationEpsilon(0.01);

    anh_ndt.setInputTarget(cloud_match);
    anh_ndt.setInputSource(cloud_in);

    Eigen::Translation3f init_translation(0, 0, 0);
    Eigen::AngleAxisf init_rotation_z(-result.second, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z) * Eigen::Matrix4f::Identity();

    anh_ndt.align(init_guess);
    Eigen::Matrix4f trans(Eigen::Matrix4f::Identity());
    trans = anh_ndt.getFinalTransformation();

    Eigen::Vector3d t(trans(0, 3), trans(1, 3), trans(2, 3));
    Eigen::Matrix3d R;
    R << trans(0, 0), trans(0, 1), trans(0, 2), trans(1, 0), trans(1, 1), trans(1, 2), trans(2, 0), trans(2, 1), trans(2, 2);
    Eigen::Quaterniond q;
    q = R;

    q_out = q_match * q;
    t_out = t_match + t;

    //DEBUG
    // printf("init found x : %.2lf, y :%.2lf \n", t_out[0], t_out[1]);
    // printf("%.4f, %.4f, %.4f, %.4f \n", trans(0,0), trans(0,1), trans(0,2), trans(0,3));
    // printf("%.4f, %.4f, %.4f, %.4f \n", trans(1,0), trans(1,1), trans(1,2), trans(1,3));
    // printf("%.4f, %.4f, %.4f, %.4f \n", trans(2,0), trans(2,1), trans(2,2), trans(2,3));
    // printf("%.4f, %.4f, %.4f, %.4f \n", trans(3,0), trans(3,1), trans(3,2), trans(3,3));
}

void map_manager::generate_submap()
{
    // submap size 100m * 100m
    for (uint32_t i = 0; i < map_->points.size(); i++)
    {
        double px = map_->points[i].x;
        double py = map_->points[i].y;

        int x = floor(px / 60);
        int y = floor(py / 60);

        string str = std::to_string(x) + "_" + std::to_string(y);

        it_ = map_name_cloud_.find(str);

        if (it_ != map_name_cloud_.end())
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp = it_->second;
            tmp->points.push_back(map_->points[i]);
        }

        else
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
            tmp->points.push_back(map_->points[i]);
            map_name_cloud_.insert(std::pair<string, pcl::PointCloud<pcl::PointXYZI>::Ptr>(str, tmp));
        }
    }

    // to pcd file
    for (it_ = map_name_cloud_.begin(); it_ != map_name_cloud_.end(); it_++)
    {
        string str = it_->first;
        string file_path = submap_path_ + "/" + str + ".pcd";

        submap_name_.insert(str);

        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp = it_->second;
        tmp->height = 1;
        tmp->width = tmp->points.size();
        // pcl::io::savePCDFileASCII(file_path, *tmp);

        printf("submap to pcd : %s \n", file_path.c_str());
    }
}

void map_manager::generate_matchcloud(const double &pose_x, const double &pose_y, bool &isNew, pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud)
{
    while (1)
    {

        int center_x = floor(pose_x / 60);
        int center_y = floor(pose_y / 60);
        if (center_x == map_centerx_ && center_y == map_centery_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        match_cloud->clear();
        printf("generate new match map and center index : %d_%d \n", center_x, center_y);

        int x = center_x;
        int y = center_y;

        vector<int> dt = {-1, 0, 1};

        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
        for (uint32_t i = 0; i < dt.size(); i++)
        {
            x = center_x + dt[i];
            for (uint32_t j = 0; j < dt.size(); j++)
            {
                y = center_y + dt[j];
                string str = std::to_string(x) + "_" + std::to_string(y);
                if (submap_name_.find(str) != submap_name_.end())
                {
                    string path = submap_path_ + "/" + str + ".pcd";
                    pcl::io::loadPCDFile<pcl::PointXYZI>(path, *tmp);
                    *match_cloud += *tmp;
                }
            }
        }

        map_centerx_ = center_x;
        map_centery_ = center_y;

        isNew = true;
    }
}