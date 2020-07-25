import numpy as np 
import sys
import math
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print('Please input sequence number as the first argument.')
    exit(1)
sequence = int(sys.argv[1])

def load_matrix(line):
    array = np.array([float(x) for x in line.split(' ')]).reshape(3, 4).tolist()
    array.append([0, 0, 0, 1])
    return np.array(array)

with open('TEMP/MATRICES/{:02}.txt'.format(sequence)) as our_poses_file:
    with open('../dataset/poses/{:02}.txt'.format(sequence)) as gt_poses_file:
        our_poses = our_poses_file.readlines()
        gt_poses = gt_poses_file.readlines()
        errors = []
        t_errors = []
        for i in range(len(our_poses) - 1):
            our_pos_1 = load_matrix(our_poses[i])
            our_pos_2 = load_matrix(our_poses[i + 1])
            gt_pos_1 = load_matrix(gt_poses[i])
            gt_pos_2 = load_matrix(gt_poses[i + 1])

            pose_delta_gt = np.matmul(np.linalg.inv(gt_pos_1), gt_pos_2)
            pose_delta_our = np.matmul(np.linalg.inv(our_pos_1), our_pos_2)
            pose_error = np.matmul(np.linalg.inv(pose_delta_our), pose_delta_gt) 

            # https://github.com/alexkreimer/odometry/blob/master/devkit/cpp/evaluate_odometry.cpp#L67-L80
            a = pose_error[0][0]
            b = pose_error[1][1]
            c = pose_error[2][2]
            d = 0.5 * (a + b + c - 1.0)

            dx = pose_error[0][3]
            dy = pose_error[1][3]
            dz = pose_error[2][3]

            r_err = math.acos(max(min(d, 1.0), -1.0))
            t_err = math.sqrt(dx * dx + dy * dy + dz * dz)
            errors.append(r_err)
            t_errors.append(t_err)

print('rad/frame percent |', sum(errors)/len(errors), '%')
print('total             |', len(errors))
print('above half        |', len([x for x in errors if x > 0.5]))
print('above one         |', len([x for x in errors if x > 1]))
print('above two         |', len([x for x in errors if x > 2]))
print('above three       |', len([x for x in errors if x > 3]))
plt.plot(errors)
plt.xlabel('Frame')
plt.ylabel('Error [rad]')
plt.title('Relative Rotation Error')
plt.savefig('rot-rpe.svg')

plt.clf()

print('m/frame percent   |', sum(t_errors)/len(t_errors), '%')                
print('total             |', len(t_errors))
plt.plot(t_errors)
plt.xlabel('Frame')
plt.ylabel('Error [m]')
plt.title('Relative Translation Error')
plt.savefig('trans-rpe.svg')
