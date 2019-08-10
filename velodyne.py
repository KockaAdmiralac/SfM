from plyfile import PlyData, PlyElement
import numpy as np
import os
import pykitti

points = []
ds = pykitti.odometry('../dataset', '00')
for i in range(100):
    for point in ds.get_velo(i):
        point[3] = 1
        tmat = np.matmul(ds.poses[i][:3], ds.calib.T_cam0_velo)
        mat = np.matmul(tmat, point)
        points.append(mat)
ply_points = np.array([tuple(x) for x in points], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
ply_element = PlyElement.describe(ply_points, 'vertex')
PlyData([ply_element]).write('velodyne.ply')
