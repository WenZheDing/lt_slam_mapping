#include <ros/ros.h>

#include <location_msgs/RTK.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <boost/foreach.hpp>
#include <unistd.h>
#include <string>
#include <mutex>

#include <eigen3/Eigen/Dense>

#include "func.h"
#include "tictoc.h"

struct imu_data
{
    double time;

    double accel_x;
    double accel_y;
    double accel_z;

    double angrate_x;
    double angrate_y;
    double angrate_z;

    double velocity_x;
    double velocity_y;
    double velocity_z;

    double angle_x;
    double angle_y;
    double angle_z;
};
class imuOdometry
{
public:
    imuOdometry(ros::NodeHandle nh2) : nh(nh2)
    {
        subLidarOdo = nh.subscribe<geometry_msgs::PoseStamped>("/odometry", 1, &imuOdometry::lidarOdoHandler, this);
        subImuRaw = nh.subscribe<location_msgs::RTK>("/rtk_data", 1, &imuOdometry::imuRawHandler, this);

        pubOdom = nh.advertise<geometry_msgs::PoseStamped>("/imu_odometry", 5);
        pubPath = nh.advertise<nav_msgs::Path>("/imu_path", 5);

        q_imu = Eigen::Quaterniond(1, 0, 0, 0);
        t_imu = Eigen::Vector3d(0, 0, 0);
        gravity = Eigen::Vector3d(0, 0, 9.81);

        q_lidar = Eigen::Quaterniond(1, 0, 0, 0);
        t_lidar = Eigen::Vector3d(0, 0, 0);
        time_lidar = 0;

        q_lidar_last = Eigen::Quaterniond(1, 0, 0, 0);
        t_lidar_last = Eigen::Vector3d(0, 0, 0);
        time_lidar_last = 0;

        imu_cur.time = 0;
        imu_cur.accel_x = 0;
        imu_cur.accel_y = 0;
        imu_cur.accel_z = 0;

        imu_cur.angrate_x = 0;
        imu_cur.angrate_y = 0;
        imu_cur.angrate_z = 0;

        imu_cur.velocity_x = 0;
        imu_cur.velocity_y = 0;
        imu_cur.velocity_z = 0;

        imu_cur.angle_x = 0;
        imu_cur.angle_y = 0;
        imu_cur.angle_z = 0;
    }

    void imuRawHandler(const location_msgs::RTKConstPtr &msg)
    {   
        mtx.lock();
        Eigen::Vector3d imu_gravity = q_imu * gravity;
        printf("gravity : %.3lf, %.3lf, %.3lf\n", imu_gravity(0), imu_gravity(1), imu_gravity(2));

        imu_cur.accel_x = msg->AccelRawY - imu_gravity(0);
        imu_cur.accel_y = -msg->AccelRawX - imu_gravity(1);
        imu_cur.accel_z = msg->AccelRawZ - imu_gravity(2);
        printf("accel : %.3lf, %.3lf, %.3lf\n", imu_cur.accel_x, imu_cur.accel_y, imu_cur.accel_z);

        imu_cur.angrate_x = msg->AngRateRawY;
        imu_cur.angrate_y = -msg->AngRateRawX;
        imu_cur.angrate_z = msg->AngRateRawZ;

        if (imu_cur.time <= 0.01)
        {
            imu_cur.time = msg->stamp.toSec();
            mtx.unlock();
            return;
        }

        double time = msg->stamp.toSec();
        double dt = time - imu_cur.time;

        t_imu(0) = t_imu(0) + imu_cur.velocity_x * dt + 0.5 * imu_cur.accel_x * dt * dt;
        t_imu(1) = t_imu(1) + imu_cur.velocity_y * dt + 0.5 * imu_cur.accel_y * dt * dt;
        t_imu(2) = t_imu(2) + imu_cur.velocity_z * dt + 0.5 * imu_cur.accel_z * dt * dt;

        imu_cur.velocity_x = imu_cur.velocity_x + imu_cur.accel_x * dt;
        imu_cur.velocity_y = imu_cur.velocity_y + imu_cur.accel_y * dt;
        imu_cur.velocity_z = imu_cur.velocity_z + imu_cur.accel_z * dt;
        printf("velocity : %.3lf, %.3lf, %.3lf\n", imu_cur.velocity_x, imu_cur.velocity_y, imu_cur.velocity_z);

        imu_cur.angle_x = imu_cur.angle_x + imu_cur.angrate_x * dt;
        imu_cur.angle_y = imu_cur.angle_y + imu_cur.angrate_y * dt;
        imu_cur.angle_z = imu_cur.angle_z + imu_cur.angrate_z * dt;

        q_imu = Eigen::AngleAxisd(imu_cur.angle_z, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(imu_cur.angle_y, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(imu_cur.angle_x, Eigen::Vector3d::UnitX());

        imu_cur.time = time;

        printf("dt : %.3lf, x : %.3lf, y : %.3lf, z : %.3lf \n", dt, t_imu(0), t_imu(1), t_imu(2));

        geometry_msgs::PoseStamped odom;
        odom.pose.orientation.x = q_imu.x();
        odom.pose.orientation.y = q_imu.y();
        odom.pose.orientation.z = q_imu.z();
        odom.pose.orientation.w = q_imu.w();
        odom.pose.position.x = t_imu(0);
        odom.pose.position.y = t_imu(1);
        odom.pose.position.z = t_imu(2);
        odom.header.frame_id = "/map";
        odom.header.stamp = msg->stamp;
        pubOdom.publish(odom);

        path.poses.push_back(odom);
        path.header.frame_id = "/map";
        path.header.stamp = msg->stamp;
        pubPath.publish(path);
        mtx.unlock();
    }

    void lidarOdoHandler(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        mtx.lock();
        printf("get lidar odo !!!\n");
        q_lidar = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        t_lidar = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        time_lidar = msg->header.stamp.toSec();

        q_imu = q_lidar;
        t_imu = t_lidar;
        Eigen::Vector3d eulerAngle = q_imu.matrix().eulerAngles(2, 1, 0);
        imu_cur.angle_z = eulerAngle(0);
        imu_cur.angle_y = eulerAngle(1);
        imu_cur.angle_x = eulerAngle(2);

        if (time_lidar_last <= 0.01)
        {
            q_lidar_last = q_lidar;
            t_lidar_last = t_lidar;
            time_lidar_last = time_lidar;
            mtx.unlock();
            return;
        }

        double dt = time_lidar - time_lidar_last;
        imu_cur.velocity_x = (t_lidar(0) - t_lidar_last(0)) / dt;
        imu_cur.velocity_y = (t_lidar(1) - t_lidar_last(1)) / dt;
        imu_cur.velocity_z = (t_lidar(2) - t_lidar_last(2)) / dt;
        q_lidar_last = q_lidar;
        t_lidar_last = t_lidar;
        time_lidar_last = time_lidar;
        mtx.unlock();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber subLidarOdo;
    ros::Subscriber subImuRaw;

    ros::Publisher pubOdom;
    ros::Publisher pubPath;

    nav_msgs::Path path;

    imu_data imu_cur;

    Eigen::Quaterniond q_imu;
    Eigen::Vector3d t_imu;
    Eigen::Vector3d gravity;

    Eigen::Quaterniond q_lidar;
    Eigen::Vector3d t_lidar;
    double time_lidar;

    Eigen::Quaterniond q_lidar_last;
    Eigen::Vector3d t_lidar_last;
    double time_lidar_last;

    std::mutex mtx;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imuOdo");
    ros::NodeHandle nh;

    imuOdometry imu(nh);

    ros::spin();

    return 0;
}
