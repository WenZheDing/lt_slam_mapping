#!/usr/bin/env python
# license removed for brevity

import rospy
import message_filters

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import os
import sys

sys.path.append('.')
from calibration_utils import *
import time


def savePointPairsAsNumpy(velo, rs):
    np.save('right', np.array(velo))  # --------------left/right-----------------
    np.save('rs_right', np.array(rs))  # --------------left/right-----------------

def callback(velo_msg, rs_msg):
    # velo_raw = numpify_velo(velo_msg)
    # ind = np.where(velo_raw[:, 3] > 100)
    # velo_filtered = velo_raw[ind[0], :3]
    # msg = getPointcloudMsg(velo_filtered, velo_msg.header)
    # pubfiltered_velo.publish(msg)
    velo_raw = numpify_rs(velo_msg)
    ind = np.where(velo_raw[:, 3] > 100)
    velo_filtered = velo_raw[ind[0], :3]
    dist = np.sqrt(np.power(velo_filtered[:, 0],2) + np.power(velo_filtered[:, 1],2))
    ind = np.where(dist > 2)
    velo_filtered = velo_filtered[ind[0], :3]
    msg = getPointcloudMsg(velo_filtered, velo_msg.header)
    pubfiltered_velo.publish(msg)


    rs_raw = numpify_rs(rs_msg)
    ind = np.where(rs_raw[:, 3] > 100)
    rs_filtered = rs_raw[ind[0], :3]
    dist = np.sqrt(np.power(rs_filtered[:, 0],2) + np.power(rs_filtered[:, 1],2))
    ind = np.where(dist > 2)
    rs_filtered = rs_filtered[ind[0], :3]
    msg = getPointcloudMsg(rs_filtered, rs_msg.header)
    pubfiltered_rs.publish(msg)

    if velo_filtered.shape[0] < 30:
        print("velo points: {}".format(velo_filtered.shape[0]))
        return
    _, velo_centroid = ransac(velo_filtered, threshold=0.03)
    if velo_centroid.shape[0] < 15:
        return
    #velo_centroid = (transform_velo @ toN4(velo_centroid[:, :3]).T).T[:, :3]
    velo_centroid = np.matmul(transform_velo,toN4(velo_centroid[:, :3]).T).T[:, :3]
    x_max = np.max(velo_centroid[:, 0])
    x_min = np.min(velo_centroid[:, 0])
    y_max = np.max(velo_centroid[:, 1])
    y_min = np.min(velo_centroid[:, 1])
    z = np.mean(velo_centroid[:, 2])
    if x_max-x_min>1.5 or y_max-y_min>1.5:
        return
    velo_centroid=np.array([(x_max+x_min)/2, (y_max+y_min)/2, z])


    if rs_filtered.shape[0] < 60:
        print("rs points: {}".format(rs_filtered.shape[0]))
        return
    _, rs_centroid = ransac(rs_filtered, threshold=0.03)
    if rs_centroid.shape[0] < 45:
    	print("rs_center x < 45")
    	return
    #rs_centroid = (transform_rs @ rs_centroid[:, :3].T).T[:, :3]
    rs_centroid = np.matmul(transform_rs,rs_centroid[:, :3].T).T[:, :3]
    x_max = np.max(rs_centroid[:, 0])
    x_min = np.min(rs_centroid[:, 0])
    y_max = np.max(rs_centroid[:, 1])
    y_min = np.min(rs_centroid[:, 1])
    z = np.mean(rs_centroid[:, 2])
    if x_max - x_min > 1.5 or y_max - y_min > 1.5:
        return
    rs_centroid = np.array([(x_max + x_min) / 2, (y_max + y_min) / 2, z])

    velo.append(velo_centroid)
    rs.append(rs_centroid)
    savePointPairsAsNumpy(velo, rs)

    ps = PoseStamped()

    velo_path.header = velo_msg.header
    ps.header = velo_msg.header
    ps.pose.position.x = velo_centroid[0]
    ps.pose.position.y = velo_centroid[1]
    ps.pose.position.z = velo_centroid[2]
    velo_path.poses.append(ps)
    pubPath_velo.publish(velo_path)

    rs_path.header = rs_msg.header
    ps.header = rs_msg.header
    ps.pose.position.x = rs_centroid[0]
    ps.pose.position.y = rs_centroid[1]
    ps.pose.position.z = rs_centroid[2]
    rs_path.poses.append(ps)
    pubPath_rs.publish(rs_path)


def cb_velo(rs_msg):
    rs_raw = numpify_rs(rs_msg)
    ind = np.where(rs_raw[:, 3] > 100)
    rs_filtered = rs_raw[ind[0], :3]
    dist = np.sqrt(np.power(rs_filtered[:, 0], 2) + np.power(rs_filtered[:, 1], 2))
    ind = np.where(dist < 10)
    rs_filtered = rs_filtered[ind[0], :3]
    msg = getPointcloudMsg(rs_filtered, rs_msg.header)
    pubfiltered_rs.publish(msg)

    if rs_filtered.shape[0] < 60:
        print("rs points: {}".format(rs_filtered.shape[0]))
        return
    _, rs_centroid = ransac(rs_filtered, threshold=0.03)
    if rs_centroid.shape[0] < 45:
        print("ransac rs points: {}".format(rs_filtered.shape[0]))
        return
    #rs_centroid = (transform_rs @ toN4(rs_centroid[:, :3]).T).T[:, :3]
    rs_centroid = np.matmul(transform_rs,toN4(rs_centroid[:, :3]).T).T[:, :3]
    x_max = np.max(rs_centroid[:, 0])
    x_min = np.min(rs_centroid[:, 0])
    y_max = np.max(rs_centroid[:, 1])
    y_min = np.min(rs_centroid[:, 1])
    z = np.mean(rs_centroid[:, 2])
    if x_max - x_min > 1.5 or y_max - y_min > 1.5:
        return
    rs_centroid = np.array([(x_max + x_min) / 2, (y_max + y_min) / 2, z])

    rs.append(rs_centroid)

    ps = PoseStamped()

    rs_path.header = rs_msg.header
    ps.header = rs_msg.header
    ps.pose.position.x = rs_centroid[0]
    ps.pose.position.y = rs_centroid[1]
    ps.pose.position.z = rs_centroid[2]
    rs_path.poses.append(ps)
    pubPath_rs.publish(rs_path)


velo_path = Path()
rs_path = Path()
velo = []
rs = []
pubPath_velo = rospy.Publisher('/path_velo', Path, queue_size=1)
pubPath_rs = rospy.Publisher('/path_rs', Path, queue_size=1)

pubfiltered_velo = rospy.Publisher('/filtered_velo', PointCloud2, queue_size=1)
pubfiltered_rs = rospy.Publisher('/filtered_rs', PointCloud2, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('record')
    # rospy.Subscriber('/rslidar_points', PointCloud2, cb_velo)
    transform_velo = transform_velo_right  # --------------left/right-----------------
    t1 = message_filters.Subscriber("/front_right/rslidar_points", PointCloud2)  # --------------left/right-----------------
    t2 = message_filters.Subscriber("/front_left/rslidar_points", PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1)
    ts.registerCallback(callback)
    rospy.spin()
