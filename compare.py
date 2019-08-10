import matplotlib.pyplot as plt
import pykitti
import re
import sys

if len(sys.argv) < 2:
    print('Please input sequence number as the first argument.')
    exit(1)

arrays = re.compile(r'\[([^\]]*)\]', re.M | re.S)

dataset = pykitti.odometry('../dataset', '00')
dataset_points = []
for pose in dataset.poses:
    point = []
    for row in pose:
        point.append(row[3])
    dataset_points.append(point)

with open('TEMP/POSITIONS/{:02}.txt'.format(int(sys.argv[1]))) as file:
    text = ''.join(file.readlines())
    points = []
    for match in re.findall(arrays, text):
        rows = match.split(';')
        point = []
        for row in rows:
            a = [float(s) for s in row.split(',')]
            point.append(a[3])
        points.append(point)
    plt.plot([point[0] for point in points], [point[2] for point in points])
    plt.plot([point[0] for point in dataset_points], [point[2] for point in dataset_points])
    plt.show()
