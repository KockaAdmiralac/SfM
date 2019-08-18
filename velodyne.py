from plyfile import PlyData, PlyElement
import numpy as np
import os
import pykitti

ds = pykitti.odometry('../dataset', '00')
for i in range(len(ds.timestamps)):
    print('Collecting point cloud', i, '     ', end='\r')
    points = []
    for point in ds.get_velo(i):
        point[3] = 1
        tmat = np.matmul(ds.poses[i][:3], ds.calib.T_cam0_velo)
        mat = np.matmul(tmat, point)
        points.append(mat)
    ply_points = np.array([tuple(x) for x in points], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    ply_element = PlyElement.describe(ply_points, 'vertex')
    PlyData([ply_element]).write('TEMP/VELODYNE/{:04}.ply'.format(i))
