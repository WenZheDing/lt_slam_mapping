#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>

using namespace std;
using namespace cv;
ros::Time rstime;

void visulization(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    const Mat &R, const Mat &t);

void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t);

void bundleAdjustment(
    const vector<Point3f> &points_1,
    const vector<Point3f> &points_2,
    Mat &R, Mat &t);

void compute_error(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    const Mat &R, const Mat &t);

vector<double> strsplit(string line, string delimiter);

void LLH_to_3Dpoint(const vector<double> &vlatitude,
                    const vector<double> &vlongitude,
                    const vector<double> &vheight,
                    vector<Point3f> &points);

void loadData(string filename, int num, vector<Point3f> &points_1, vector<Point3f> &points_2);

void tofile(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    const Mat &R, const Mat &t);

void rsLidarCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    rstime = msg->header.stamp;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "usage: pose_estimation_3d3d data_path data_num" << endl;
        return 1;
    }

    ros::init(argc, argv, "pose3d3d");
    ros::NodeHandle nh("~");

    ros::Publisher pubLidarCloud = nh.advertise<sensor_msgs::PointCloud2>("/Lidar_points", 2);
    ros::Publisher pubRtkCloud = nh.advertise<sensor_msgs::PointCloud2>("/Rtk_points", 2);
    ros::Publisher pubTransCloud = nh.advertise<sensor_msgs::PointCloud2>("/Trans_points", 2);

    ros::Subscriber subrsLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, rsLidarCloudHandler);

    vector<Point3f> pts1, pts2;
    int num = atoi(argv[2]);
    loadData(argv[1], num, pts1, pts2);

    Mat R, t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout << "ICP via SVD results: " << endl;
    cout << "R = " << R << endl;
    cout << "t = " << t << endl;
    cout << "R_inv = " << R.t() << endl;
    cout << "t_inv = " << -R.t() * t << endl;
    compute_error(pts1, pts2, R, t);

    Eigen::Matrix3d R_mat;
    R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    Eigen::Vector3d t_mat;
    t_mat << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);

    Eigen::Quaternion<double> q_mat(R_mat);
    printf("x:%lf, y:%lf, z:%lf, w:%lf\n", q_mat.x(), q_mat.y(), q_mat.z(), q_mat.w());
    printf("????????????\n");
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr pRtkCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pTransCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pLidarCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < pts1.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = pts1[i].x;
        point.y = pts1[i].y;
        point.z = pts1[i].z;
        pLidarCloud->push_back(point);

        Eigen::Vector3d pts(pts2[i].x, pts2[i].y, pts2[i].z);
        pts = R_mat * pts + t_mat;
        point.x = pts.x();
        point.y = pts.y();
        point.z = pts.z();
        pTransCloud->push_back(point);

        point.x = pts2[i].x;
        point.y = pts2[i].y;
        point.z = pts2[i].z;
        pRtkCloud->push_back(point);
    }

    
    Eigen::Quaterniond q2;
    q2 =  R_mat;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));
    q.setW(q2.w());
    q.setX(q2.x());
    q.setY(q2.y());
    q.setZ(q2.z());
    transform.setRotation(q);

    while (ros::ok())
    {
        sensor_msgs::PointCloud2 LidarCloud;
        pcl::toROSMsg(*pLidarCloud, LidarCloud);
        LidarCloud.header.stamp = ros::Time().now();
        LidarCloud.header.frame_id = "/rslidar";
        pubLidarCloud.publish(LidarCloud);

        sensor_msgs::PointCloud2 TransCloud;
        pcl::toROSMsg(*pTransCloud, TransCloud);
        TransCloud.header.stamp = LidarCloud.header.stamp;
        TransCloud.header.frame_id = "/rslidar";
        pubTransCloud.publish(TransCloud);

        sensor_msgs::PointCloud2 RtkCloud;
        pcl::toROSMsg(*pRtkCloud, RtkCloud);
        RtkCloud.header.stamp = LidarCloud.header.stamp;
        RtkCloud.header.frame_id = "/rtk";
        pubRtkCloud.publish(RtkCloud);

        br.sendTransform(tf::StampedTransform(transform, LidarCloud.header.stamp, "/rslidar", "/rtk"));
        usleep(1e4); 
    }

    // visulization(pts1, pts2, R, t);
    // tofile(pts1, pts2, R, t);
}

void visulization(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    const Mat &R, const Mat &t)
{
    Eigen::Matrix3d R_mat;
    R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    Eigen::Vector3d t_mat;
    t_mat << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pRtkCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pLidarCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pTransCloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < pts1.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = pts1[i].x;
        point.y = pts1[i].y;
        point.z = pts1[i].z;
        pLidarCloud->push_back(point);

        point.x = pts2[i].x;
        point.y = pts2[i].y;
        point.z = pts2[i].z;
        pRtkCloud->push_back(point);

        Eigen::Vector3d pts(pts2[i].x, pts2[i].y, pts2[i].z);
        pts = R_mat * pts + t_mat;
        point.x = pts.x();
        point.y = pts.y();
        point.z = pts.z();
        pTransCloud->push_back(point);
    }

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing rtk cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        rtk_color(pRtkCloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(pRtkCloud, rtk_color, "rtk cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "rtk cloud");

    // Coloring and visualizing lidar cloud (blue).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        lidar_color(pLidarCloud, 0, 0, 255);
    viewer_final->addPointCloud<pcl::PointXYZ>(pLidarCloud, lidar_color, "lidar cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "lidar cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(pTransCloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(pTransCloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    Point3f p1, p2; // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0)
    {
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
    }

    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
         R_(1, 0), R_(1, 1), R_(1, 2),
         R_(2, 0), R_(2, 1), R_(2, 2));
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

void compute_error(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    const Mat &R, const Mat &t)
{
    cout << "compute error : " << endl;
    Eigen::Matrix3d R_mat;
    R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    Eigen::Vector3d t_mat;
    t_mat << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);

    double error = 0;
    for (int i = 0; i < pts1.size(); i++)
    {
        Eigen::Vector3d pts(pts2[i].x, pts2[i].y, pts2[i].z);
        pts = R_mat * pts + t_mat;
        Point3f dp(pts1[i].x - pts[0], pts1[i].y - pts[1], pts1[i].z - pts[2]);
        printf("%f,%f,%f —— ", pts1[i].x, pts1[i].y, pts1[i].z);
        printf("%f,%f,%f \n", pts2[i].x, pts2[i].y, pts2[i].z);
        printf("%f,%f,%f —— %f\n", pts[0], pts[1], pts[2], sqrt(dp.x * dp.x + dp.y * dp.y + dp.z * dp.z));
        error += sqrt(dp.x * dp.x + dp.y * dp.y + dp.z * dp.z);
    }

    error /= pts1.size();
    cout << "project error : " << error << endl;
}

vector<double> strsplit(string line, string delimiter)
{
    int pos = 0;
    string token;
    vector<double> data;
    while ((pos = line.find(delimiter)) != string::npos)
    {
        token = line.substr(0, pos);
        data.push_back(stod(token));
        line.erase(0, pos + delimiter.length());
    }
    return data;
}

void loadData(
    string filename,
    int num,
    vector<Point3f> &points_1, vector<Point3f> &points_2)
{
    ifstream infile;
    vector<double> vlatitude;
    vector<double> vlongitude;
    vector<double> vheight;

    infile.open(filename, ios::in);
    int line_num = 0;
    if (!infile.is_open())
    {
        cout << "open file failed" << endl;
    }
    else
    {
        cout << "open file success" << endl;
        string line;
        while (getline(infile, line))
        {
            vector<double> point = strsplit(line, ",");

            if (line_num < num)
            {
                Point3f point_raw(point[0], point[1], point[2]);
                points_1.push_back(point_raw);
            }
            else
            {   
                vlatitude.push_back(point[0]);
                vlongitude.push_back(point[1]);
                vheight.push_back(point[2]);
                printf("tmp: %f,%f,%f \n", point[0], point[1], point[2]);
            }
            line_num++;
        }
        infile.close();
        LLH_to_3Dpoint(vlatitude, vlongitude, vheight, points_2);
    }

    // check
    printf("size of points1 : %d \n", points_1.size());
    printf("size of points2 : %d \n", points_2.size());
    if (points_1.size() == points_2.size())
    {
        for (int i = 0; i < points_1.size(); i++)
        {
            printf("x1:%.4f,y1:%.4f,z1:%.4f——x2:%.4f,y2:%.4f,z2:%.4f\n",
                   points_1[i].x, points_1[i].y, points_1[i].z,
                   points_2[i].x, points_2[i].y, points_2[i].z);
        }
    }
}

void LLH_to_3Dpoint(const vector<double> &vlatitude,
                    const vector<double> &vlongitude,
                    const vector<double> &vheight,
                    vector<Point3f> &points)
{
    Eigen::Matrix3d R_ENU2IMU;
    Eigen::Vector3d t_ECEF2ENU(0, 0, 0);
    Eigen::Vector3d t_ECEF(0, 0, 0);
    Eigen::Vector3d t_ENU(0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Quaternion<double> q(1, 0, 0, 0);
    double e = 0.0818191908425;
    double R = 6378137;
    double torad = M_PI / 180;
    double g = 9.7964;
    double Re = 0;

    double latitude = vlatitude[0], longitude = vlongitude[0], height = vheight[0];

    R_ENU2IMU = Eigen::AngleAxisd(16.377 * torad, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(-0.38 * torad, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(-0.69 * torad, Eigen::Vector3d::UnitX());
    Re = R / sqrt(1 - e * e * sin(latitude * torad) * cos(longitude * torad));
    t_ECEF2ENU[0] = (Re + height) * cos(latitude * torad) * cos(longitude * torad);
    t_ECEF2ENU[1] = (Re + height) * cos(latitude * torad) * sin(longitude * torad);
    t_ECEF2ENU[2] = (Re * (1 - e * e) + height) * sin(latitude * torad);

    for (int i = 1; i < vlatitude.size(); i++)
    {
        latitude = vlatitude[i], longitude = vlongitude[i], height = vheight[i];

        t_ECEF[0] = (Re + height) * cos(latitude * torad) * cos(longitude * torad) - t_ECEF2ENU[0];
        t_ECEF[1] = (Re + height) * cos(latitude * torad) * sin(longitude * torad) - t_ECEF2ENU[1];
        t_ECEF[2] = (Re * (1 - e * e) + height) * sin(latitude * torad) - t_ECEF2ENU[2];

        t_ENU[0] = -sin(longitude * torad) * t_ECEF[0] + cos(longitude * torad) * t_ECEF[1];
        t_ENU[1] = -sin(latitude * torad) * cos(longitude * torad) * t_ECEF[0] - sin(latitude * torad) * sin(longitude * torad) * t_ECEF[1] + cos(latitude * torad) * t_ECEF[2];
        t_ENU[2] = cos(latitude * torad) * cos(longitude * torad) * t_ECEF[0] + cos(latitude * torad) * sin(longitude * torad) * t_ECEF[1] + sin(latitude * torad) * t_ECEF[2];

        t = R_ENU2IMU * t_ENU;

        Point3f point(t[0], t[1], t[2]);
        points.push_back(point);
    }
}

void tofile(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    const Mat &R, const Mat &t)
{
    ofstream outfile;
    outfile.open("out.txt", ios::out);
    for (int i = 0; i < pts2.size(); i++)
        outfile << setiosflags(ios::fixed) << setprecision(4) << pts2[i].x << "," << pts2[i].y << "," << pts2[i].z << "," << endl;
    outfile.close();
}