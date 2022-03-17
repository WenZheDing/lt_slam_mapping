#!/usr/bin/env python3
# license removed for brevity
import numpy as np
import rospy
import os
import sys
import message_filters

sys.path.append('.')

from calibration_utils import *
import time
import pickle

R = None
t = None


def callback(velo_msg, rs_msg):
    velo_raw = numpify_rs(velo_msg)
    rs_raw = numpify_rs(rs_msg)

    # rs_tf = (transform_rs@rs_raw[:, :3].T).T

    velo_tf1 = np.matmul(transform_rs_inv,transform_velo2)
    velo_tf2 = np.matmul(velo_tf1,transform_velo)
    velo_tf = np.matmul(velo_tf2, toN4(velo_raw[:,:3]).T).T[:,:3]

    # velo_tf = (transform_rs_inv @ transform_velo2 @ transform_velo @ toN4(velo_raw[:, :3]).T).T[:, :3]

    msg = getPointcloudMsg(rs_raw, rs_msg.header)
    pubTransformed_rs.publish(msg)
    msg = getPointcloudMsg(velo_tf, rs_msg.header)
    pubTransformed_velo.publish(msg)


if __name__ == '__main__':
    rospy.init_node('calibrate')
    pubTransformed_velo = rospy.Publisher('/velo_transformed', PointCloud2, queue_size=1)
    pubTransformed_rs = rospy.Publisher('/rs_transformed', PointCloud2, queue_size=1)


    transform_rs_inv = np.eye(4)
    transform_rs_inv[:3, :3] = np.linalg.inv(transform_rs)
    transform_velo=transform_velo_right   # --------------left/right-----------------

    velo = np.load('right.npy')    # --------------left/right-----------------
    rs = np.load('rs_right.npy')  # --------------left/right-----------------
    R, t = getTransformFromAtoB(np.matmul(transform_velo , toN4(velo[:, :3]).T).T[:, :2], np.matmul(transform_rs , rs[:, :3].T).T[:, :2])
    transform_velo2 = np.eye(4)
    transform_velo2[:2, :2] = R
    transform_velo2[0, 3] = t[0]
    transform_velo2[1, 3] = t[1]

    print(np.matmul(transform_rs_inv , np.matmul(transform_velo2 , transform_velo)))
    print(np.matmul(np.matmul(transform_rs_inv,transform_velo2),transform_velo))

    t1 = message_filters.Subscriber("/front_right/rslidar_points", PointCloud2)  # --------------left/right-----------------
    t2 = message_filters.Subscriber("/front_left/rslidar_points", PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1)
    ts.registerCallback(callback)
    rospy.spin()
