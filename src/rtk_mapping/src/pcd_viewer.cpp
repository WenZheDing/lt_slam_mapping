
#include <iostream>
#include <thread>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "map_manager.h"

int main(int argc, char **argv)
{

    if (argc != 2)
    {
        printf("usage: pcl_viewer pcd_path\n");
        return 1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud) == -1)
        {
            printf("can't read open pcd file \n");
            exit(-1);
        }


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> zaxis_distribution(cloud, "z");

    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "sample cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}