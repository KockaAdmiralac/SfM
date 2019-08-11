from plyfile import PlyData, PlyElement
import numpy as np
import os
import pykitti
import sys

if len(sys.argv) < 2:
    print('Sequence number is a required parameter.')
    exit(1)
sequence = int(sys.argv[1])

ds = pykitti.odometry('../dataset', '{:02}'.format(sequence))
files = os.listdir('TEMP/TRIANGULATION/{:02}'.format(sequence))
points = []
for i in range(len(files)):
    ply = PlyData.read('TEMP/TRIANGULATION/{:02}/{:02}.ply'.format(sequence, i))
    for point in ply['vertex'].data:
        points.append(np.matmul(ds.poses[i][:3], [point[0], point[1], point[2], 1]))
    print('Currently on point cloud', i, end='\r')
ply_points = np.array([tuple(x) for x in points], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
ply_element = PlyElement.describe(ply_points, 'vertex')
PlyData([ply_element]).write('merge.ply')
