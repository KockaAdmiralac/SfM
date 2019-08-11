import numpy as np 
import sys
import math
import matplotlib.pyplot as plt
if len(sys.argv) < 2:
    print('Please input sequence number as the first argument.')
    exit(1)


with open('TEMP/MATRICES/{:02}.txt'.format(int(sys.argv[1]))) as ourPosesFile:
    with open('../dataset/poses/{:02}.txt'.format(int(sys.argv[1]))) as gtPosesFile:
        errorSum = 0
        total = 0
        aboveHalf = 0
        aboveOne = 0
        aboveTwo = 0
        aboveThree = 0
        ourPoses = ourPosesFile.readlines()
        gtPoses = gtPosesFile.readlines()
        errors = []
        for i in range(len(ourPoses)-1):
            array = ourPoses[i].split(' ')
            array1 = [float(x) for x in array]
            array1 = np.array(array1).reshape(3, 4)
            array2 = array1.tolist()
            array2.append([0,0,0,1])
            ourPos1 = np.array(array2)

            array = ourPoses[i+1].split(' ')
            array1 = [float(x) for x in array]
            array1 = np.array(array1).reshape(3, 4)
            array2 = array1.tolist()
            array2.append([0,0,0,1])
            ourPos2 = np.array(array2)

            array = gtPoses[i].split(' ')
            array1 = [float(x) for x in array]
            array1 = np.array(array1).reshape(3, 4)
            array2 = array1.tolist()
            array2.append([0,0,0,1])
            gtPos1 = np.array(array2)

            array = gtPoses[i+1].split(' ')
            array1 = [float(x) for x in array]
            array1 = np.array(array1).reshape(3, 4)
            array2 = array1.tolist()
            array2.append([0,0,0,1])
            gtPos2 = np.array(array2)

            pose_delta_gt = np.matmul(np.linalg.inv(gtPos1),gtPos2)
            pose_delta_our = np.matmul(np.linalg.inv(ourPos1), ourPos2)
            pose_error = np.matmul(np.linalg.inv(pose_delta_our),pose_delta_gt) 

            a = pose_error[0][0]
            b = pose_error[1][1]
            c = pose_error[2][2]
            d = 0.5*(a+b+c-1.0)
            
            r_err =  math.acos(max(min(d,1.0),-1.0))
            print(r_err)
            errors.append(r_err)
            
            errorSum += r_err
            
            total += 1
            if r_err > 0.5:
                aboveHalf += 1
            if r_err > 1:
                aboveOne += 1
            if r_err > 2:
                aboveTwo += 1
            if r_err >3:
                aboveThree += 1
print("rad/m percent | ", errorSum/total, "%")                
print("total         | ",total)
print("aboveHalf     | ",aboveHalf)
print("aboveOne      | ",aboveOne)
print("aboveTwo      | ",aboveTwo)
print("aboveThree    | ",aboveThree)
plt.plot(errors)
plt.show()
