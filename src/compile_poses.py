import numpy as np
import json
import os
import sys
from kabsch import *

def transformed(pos, R, centroids):
	tranlated = pos - centroids
	rotated = np.matmul(R,tranlated.T)
	return rotated.T

if __name__ == '__main__':
	# find the data
	trial_dir = sys.argv[1]
	data_dir = os.path.join(trial_dir,'data')
	pose_dir = os.path.join(trial_dir,'poses-proposed')

	# get the files
	data_files = sorted(os.listdir(data_dir))
	pos0 = np.load(os.path.join(pose_dir,'0_pos.npy'))*1000
	pos1 = np.load(os.path.join(pose_dir,'1_pos.npy'))*1000
	pos2 = np.load(os.path.join(pose_dir,'2_pos.npy'))*1000
	pos3 = np.load(os.path.join(pose_dir,'3_pos.npy'))*1000
	pos4 = np.load(os.path.join(pose_dir,'4_pos.npy'))*1000
	pos5 = np.load(os.path.join(pose_dir,'5_pos.npy'))*1000

	# skip the first two frames
	pos0 = pos0[2:]
	pos1 = pos1[2:]
	pos2 = pos2[2:]
	pos3 = pos3[2:]
	pos4 = pos4[2:]
	pos5 = pos5[2:]

	# get the Kabsch transformation from the first N frames
	N = 1
	N = min(N,len(data_files))
	A = []
	for i in range(N):
		data = json.load(open(os.path.join(data_dir,data_files[i])))
		SR = data['state_reconstruction']
		A.extend([[SR['0']['x'],SR['0']['y'],SR['0']['z']],
				  [SR['1']['x'],SR['1']['y'],SR['1']['z']],
				  [SR['2']['x'],SR['2']['y'],SR['2']['z']],
				  [SR['3']['x'],SR['3']['y'],SR['3']['z']],
				  [SR['4']['x'],SR['4']['y'],SR['4']['z']],
				  [SR['5']['x'],SR['5']['y'],SR['5']['z']]])
		# A = np.array([[SR['0']['x'],-SR['0']['y'],SR['0']['z']],
		# 			  [SR['1']['x'],-SR['1']['y'],SR['1']['z']],
		# 			  [SR['2']['x'],-SR['2']['y'],SR['2']['z']],
		# 			  [SR['3']['x'],-SR['3']['y'],SR['3']['z']],
		# 			  [SR['4']['x'],-SR['4']['y'],SR['4']['z']],
		# 			  [SR['5']['x'],-SR['5']['y'],SR['5']['z']]])
	A = np.array(A)
	# print(A)
	# B = np.array([pos0[0,:],pos1[0,:],pos2[0,:],pos3[0,:],pos4[0,:],pos5[0,:]])
	B = []
	for i in range(N):
		B.extend([pos0[i,:],pos1[i,:],pos2[i,:],pos3[i,:],pos4[i,:],pos5[i,:]])
	B = np.array(B)
	# Rz = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
	# B2 = np.matmul(Rz,B.T)
	# B = B2.T
	# print(B)
	R,t = rigid_transform_3D(A,B)
	rmse = kabsch_rmse(A,B,R,t)
	print('RMSE: ',rmse)
	# R = np.matmul(Rz,R)

	# get the centroid at each time step
	all_points = np.zeros((len(data_files),3,6))
	all_points[:,:,0] = pos0
	all_points[:,:,1] = pos1
	all_points[:,:,2] = pos2
	all_points[:,:,3] = pos3
	all_points[:,:,4] = pos4
	all_points[:,:,5] = pos5
	centroids = np.mean(all_points, axis=2)

	# transform all the points to the world frame
	pos0_t = transformed(pos0, R, centroids)
	pos1_t = transformed(pos1, R, centroids)
	pos2_t = transformed(pos2, R, centroids)
	pos3_t = transformed(pos3, R, centroids)
	pos4_t = transformed(pos4, R, centroids)
	pos5_t = transformed(pos5, R, centroids)

	# loop through and append to the json
	for i in range(len(data_files)):
		data = json.load(open(os.path.join(data_dir,data_files[i])))
		data['raw_pose_estimation'] = {0:{'x':pos0[i,0],'y':pos0[i,1],'z':pos0[i,2]},
								   1:{'x':pos1[i,0],'y':pos1[i,1],'z':pos1[i,2]},
								   2:{'x':pos2[i,0],'y':pos2[i,1],'z':pos2[i,2]},
								   3:{'x':pos3[i,0],'y':pos3[i,1],'z':pos3[i,2]},
								   4:{'x':pos4[i,0],'y':pos4[i,1],'z':pos4[i,2]},
								   5:{'x':pos5[i,0],'y':pos5[i,1],'z':pos5[i,2]}}
		data['info'].update({'kabsch_rmse':rmse})
		data['pose_estimation'] = {0:{'x':pos0_t[i,0],'y':pos0_t[i,1],'z':pos0_t[i,2]},
								   1:{'x':pos1_t[i,0],'y':pos1_t[i,1],'z':pos1_t[i,2]},
								   2:{'x':pos2_t[i,0],'y':pos2_t[i,1],'z':pos2_t[i,2]},
								   3:{'x':pos3_t[i,0],'y':pos3_t[i,1],'z':pos3_t[i,2]},
								   4:{'x':pos4_t[i,0],'y':pos4_t[i,1],'z':pos4_t[i,2]},
								   5:{'x':pos5_t[i,0],'y':pos5_t[i,1],'z':pos5_t[i,2]}}
		json.dump(data,open(os.path.join(data_dir,data_files[i]),'w'))