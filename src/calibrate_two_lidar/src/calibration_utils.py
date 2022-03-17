import numpy as np
import math
from numpy.core.numeric import ones

from numpy.linalg.linalg import lstsq
import ros_numpy
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2

transform_velo_right = np.array([[0.999892, -0.000201233, -0.0146793, 0],
                           [-0.000201233, 0.999624, -0.0274106, 0],
                           [0.0146793, 0.0274106, 0.999516, 0.0522392],
                           [0, 0, 0, 1]])  # right
transform_velo_left = np.array([[0.999732, 0.00794733, -0.0217259, 0],
                           [0.00794733, 0.763996, 0.645172, 0],
                           [0.0217259, -0.645172, 0.763729, -0.158917],
                           [0, 0, 0, 1]])  # left
transform_rs = np.array([[0.999731, -0.0004720, 0.023174],
                         [-0.000472026, 0.999171, 0.0407152],
                         [-0.023174, -0.0407152, 0.998902]])

def toN4(pointsN3):
    return np.concatenate([pointsN3, np.ones([pointsN3.shape[0], 1])], axis=1)



def ransac(pointcloud, max_iteration=40, threshold=0.1):
    coeff_list = []
    inliers_list = []
    rate_list = []
    for i in range(max_iteration):
        selected_points = pointcloud[np.random.choice(np.arange(pointcloud.shape[0]),
                                                      3, pointcloud.shape[0]), :]
        plane_coeff = np.linalg.svd(toN4(selected_points))[-1][-1, :]
        # inliers = np.where(np.abs(plane_coeff @ toN4(pointcloud).T) < threshold)
        inliers = np.where(np.abs(np.matmul(plane_coeff , toN4(pointcloud).T) < threshold))

        coeff_list.append(plane_coeff)
        inliers_list.append(inliers)
        rate_list.append(len(inliers[0]))
    best_ind = np.argmax(rate_list)
    inlier_points = pointcloud[inliers_list[best_ind], :][0]
    # showPointCloud(inlier_points/100)
    # print(coeff_list[best_ind] / coeff_list[best_ind][-1])  # ax+by+cz+1=0

    lstsq_fit1 = np.linalg.inv(np.matmul(inlier_points.T , inlier_points))   # ax+by+cz=1
    lstsq_fit2 = np.matmul(inlier_points.T, np.ones([inlier_points.shape[0], 1]))  
    lstsq_fit = np.matmul(lstsq_fit1 , lstsq_fit2)         
    lstsq_fit[:3] *= np.sign(lstsq_fit[2])  # ax+by+cz+1=0
    # print("fit:{}, inlier_ratio={}".format(lstsq_fit, inlier_points.shape[0] / pointcloud.shape[0]))
    inlier_ratio = {}
    print("ransac result:{} ,inlier ratio:{}".format(lstsq_fit, inlier_points.shape[0] / pointcloud.shape[0]))
    return lstsq_fit, inlier_points


def alignZaxis(normal_vec31):
    o_vec = normal_vec31 / np.linalg.norm(normal_vec31)  # origin
    t_vec = np.array([0, 0, 1]).reshape(1,3)  # target
    o_vec.reshape(3,1)
    print(o_vec.shape)
    print(t_vec.shape)
    svd = np.linalg.svd(np.matmul(o_vec , t_vec))
    R = np.matmul(svd[0] , svd[2].T)
    '''theta = math.acos(t_vec @ o_vec)
    n_vec = np.cross(t_vec, o_vec.T)[0]  # normal
    K = np.array([[0, -n_vec[2], n_vec[1]],
                  [n_vec[2], 0, -n_vec[0]],
                  [-n_vec[1], n_vec[0], 0]])
    R_mat = np.eye(3) - math.sin(theta) * K - K.T @ K * (1 - math.cos(theta))'''
    return R.T


def rotatePointcloud(pointcloud_N3, angle_rad):  # clock-wise
    rotationMatrix = np.array([[math.cos(angle_rad), -math.sin(angle_rad), 0],
                               [math.sin(angle_rad), math.cos(angle_rad), 0],
                               [0, 0, 1]])
    return np.matmul(pointcloud_N3 , rotationMatrix)


def rotate180y(pointcloud_N3):  # clock-wise
    rotationMatrix = np.array([[-1, 0, 0],
                               [0, 1, 0],
                               [0, 0, -1]])
    return np.matmul(pointcloud_N3 , rotationMatrix)

def getTransformFromAtoB(A, B):
    A_mean = np.mean(A, axis=0)
    B_mean = np.mean(B, axis=0)
    A_n = A - A_mean
    B_n = B - B_mean
    svd = np.linalg.svd(np.matmul(B_n.T , A_n))
    R = np.matmul(svd[0] , svd[2].T)
    t = B_mean - np.matmul(R , A_mean.T).T
    '''print(R)
    print(t)
    print((R@A.T).T+t)'''
    return R, t


def numpify_velo(pc2_msg):
    pc = ros_numpy.numpify(pc2_msg)
    raw_points = np.zeros((pc.shape[0], 5))
    raw_points[:, 0] = pc['x']
    raw_points[:, 1] = pc['y']
    raw_points[:, 2] = pc['z']
    raw_points[:, 3] = pc['intensity']
    raw_points[:, 4] = pc['ring']
    return raw_points

def numpify_rs(pc2_msg):
    pc = ros_numpy.numpify(pc2_msg).flatten()
    mask=np.nonzero(pc['intensity'])
    raw_points = np.zeros((len(mask[0]), 4))
    raw_points[:, 0] = pc['x'][mask]
    raw_points[:, 1] = pc['y'][mask]
    raw_points[:, 2] = pc['z'][mask]
    raw_points[:, 3] = pc['intensity'][mask]
    return raw_points

def getPointcloudMsg(points, header):  # (N,3)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = False
    msg.data = np.asarray(points[:, :3], np.float32).tostring()
    return msg


if __name__ == '__main__':
    pointcloud = np.random.randn(50, 3)
    pointcloud[:, 2] /= 100
    pointcloud[:, 2] += 6
    '''pointcloud = load_pcd_to_ndarray("../../Hirain/pcds/000067.pcd")[:,:3]
    mask = np.isnan(pointcloud[:,0])
    mask = np.where(mask!=True)
    pointcloud=pointcloud[mask[0],:3]'''
    fit = ransac(pointcloud)
    R_mat = alignZaxis(fit)
    pointcloud = np.matmul(pointcloud , R_mat)
    fit2 = ransac(pointcloud)
    pointcloud[:, 2] += 1 / fit2[2]
