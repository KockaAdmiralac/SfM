from plyfile import PlyData, PlyElement
import numpy as np
import os
import pykitti

# points_file_path = os.path.join('..', 'dataset', 'sequences', '00', 'velodyne', '000000.bin')
# loaded_points = np.fromfile(points_file_path, dtype=(float, 3)).reshape(-1, 4)
ds = pykitti.odometry('../dataset', '00')
loaded_points = None
for i in range(10):
    if type(loaded_points) == np.ndarray:
        loaded_points = np.append(loaded_points, ds.get_velo(i), 0)
    else:
        loaded_points = ds.get_velo(i)
print(len(loaded_points))
points_without_luminance = loaded_points[:, :3]
# approx_points = points_without_luminance[points_without_luminance[:, 0] >= 1, :]
ply_points = np.array([tuple(x) for x in points_without_luminance], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
ply_element = PlyElement.describe(ply_points, 'vertex')
PlyData([ply_element]).write('velodyne.ply')
