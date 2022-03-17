#include <ros/ros.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <eigen3/Eigen/Dense>

#include <vector>
using std::vector;
using namespace gtsam;

int main(int argc, char **argv)
{

    FILE *sc_read_fp = NULL;
    sc_read_fp = fopen(argv[1], "r");

    vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> sc_q;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> sc_t;

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
            q.normalize();
            sc_t.push_back(t);
            sc_q.push_back(q);
        }
        sc_t.pop_back();
        sc_q.pop_back();
    }

    fclose(sc_read_fp);

    printf("read finished %ld - %ld\n", sc_t.size(), sc_q.size());

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    noiseModel::Diagonal::shared_ptr constraintNoise;
    noiseModel::Base::shared_ptr robustNoiseModel;

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    gtsam::Vector Vector1(6);
    Vector1 << 1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3; // rotation xyz
    odometryNoise = noiseModel::Diagonal::Variances(Vector1);

    gtsam::Vector Vector2(6);
    Vector2 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(Vector2);

    Eigen::Quaterniond q_graph(1, 0, 0, 0);
    Eigen::Vector3d t_graph(0, 0, 0);

    // first
    gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3(q_graph), Point3(t_graph)), priorNoise));
    initialEstimate.insert(0, Pose3(Rot3(q_graph), Point3(t_graph)));
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // other
    for (int i = 1; i < sc_q.size() - 1; i++)
    {
        gtsam::Pose3 poseFrom = Pose3(Rot3(q_graph), Point3(t_graph));

        t_graph = sc_t[i];
        q_graph = sc_q[i];

        gtsam::Pose3 poseTo = Pose3(Rot3(q_graph), Point3(t_graph));

        gtSAMgraph.add(BetweenFactor<Pose3>(i - 1, i, poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(i, Pose3(Rot3(q_graph), Point3(t_graph)));

        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();
    }

    // loop
    Eigen::Quaterniond q_loop = sc_q[sc_q.size() - 1];
    Eigen::Vector3d t_loop = sc_t[sc_t.size() - 1];
    
    Eigen::Quaterniond q_check = q_loop * q_graph;
    Eigen::Vector3d t_check = q_loop * t_graph + t_loop;
     printf("check : %.2lf, %.2lf, %.2lf\n", t_check[0], t_check[1], t_check[2]);
    float noiseScore = 0.5; //TODO
    gtsam::Vector Vector6(6);
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    constraintNoise = noiseModel::Diagonal::Variances(Vector6);
    robustNoiseModel = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
        gtsam::noiseModel::Diagonal::Variances(Vector6));

    gtsam::Pose3 poseFrom = Pose3(Rot3(q_check), Point3(t_check));
    gtsam::Pose3 poseTo = Pose3(Rot3(sc_q[0]), Point3(sc_t[0]));

    gtSAMgraph.add(BetweenFactor<Pose3>(sc_q.size() - 2, 0, poseFrom.between(poseTo), odometryNoise));

    isam->update(gtSAMgraph);
    for (int i = 0; i < 15; i++)
    {
        isam->update();
        isamCurrentEstimate = isam->calculateEstimate();
        int numPoses = isamCurrentEstimate.size();
        double x = isamCurrentEstimate.at<Pose3>(numPoses - 1).translation().x();
        double y = isamCurrentEstimate.at<Pose3>(numPoses - 1).translation().y();
        double z = isamCurrentEstimate.at<Pose3>(numPoses - 1).translation().z();
        printf("sam : %.2lf, %.2lf, %.2lf\n", x, y, z);
    }
    // isam->update();
    // isam->update();
    // isam->update();
    // isam->update();
    // isam->update();
    // isamCurrentEstimate = isam->calculateEstimate();
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    Eigen::Vector3d t_ref = sc_q[sc_q.size() - 1] * sc_t[sc_t.size() - 2] + sc_t[sc_t.size() - 1];
    printf("ref : %.2lf, %.2lf, %.2lf\n", t_ref[0], t_ref[1], t_ref[2]);
    double x = isamCurrentEstimate.at<Pose3>(0).translation().x();
    double y = isamCurrentEstimate.at<Pose3>(0).translation().y();
    double z = isamCurrentEstimate.at<Pose3>(0).translation().z();
    printf("ori : %.2lf, %.2lf, %.2lf\n", x, y, z);
    return 0;
}