import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from pyquaternion import Quaternion

def quatWAvgMarkley(Q, weights):
    '''
    Averaging Quaternions.

    Arguments:
        Q(ndarray): an Mx4 ndarray of quaternions.
        weights(list): an M elements list, a weight for each quaternion.
    '''

    # Form the symmetric accumulator matrix
    A = np.zeros((4, 4))
    M = Q.shape[0]
    wSum = 0

    for i in range(M):
        q = Q[i, :]
        w_i = weights[i]
        A += w_i * (np.outer(q, q)) # rank 1 update
        wSum += w_i

    # scale
    A /= wSum

    # Get the eigenvector corresponding to largest eigen value
    return np.linalg.eigh(A)[1][:, -1]

filenames = ["stat" + str(i) + ".json" for i in range(1,11)]
filepath = "../calibration"
filenames = [os.path.join(filepath,f) for f in filenames]

matrices = [json.load(open(f))['R'] for f in filenames]
# rotations = [R.from_matrix(np.array(m)) for m in matrices]
# quaternions = [r.as_quat() for r in rotations]
quaternions = np.array([Quaternion(matrix=np.array(m)).elements for m in matrices])
weights = [1]*quaternions.shape[0]

avgq = quatWAvgMarkley(quaternions,weights)
q = Quaternion(avgq)
R = q.rotation_matrix
t = [0,0,0]
json.dump({"R":np.ndarray.tolist(R),"t":t},open(os.path.join(filepath,'average.json'),'w'))