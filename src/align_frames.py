"""
This is for finding the transformation (rotation and translation) between two frames given a set of corresponding points in each frame.

Usage:
python align_frames.py data_directory frameA frameB [output_filename]

data_directory is a path to a directory full of .json files that contain the points
frameA is the key to get the points from the frame you want to rotate to (ex. "state reconstruction")
frameB is the key to get the points from the frame you want to rotate from (ex. "mocap")
output_filename is an optional argument.  If given, the rotation and translation will be saved to the specified .json file

To use the R and t that this script calculates to transfrom your points, use the function kabsch.kabsch_transformed
"""
import numpy as np
from math import sqrt
import sys
import os
import json
import matplotlib.pyplot as plt
# from matplotlib.figure import Figure
from kabsch import *

def dist(points):
	point1,point2 = points
	x1,y1,z1 = point1
	x2,y2,z2 = point2
	return sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

def get_points(dic,pair):
	# gets the points from a dictionary of indexed points given the pair of points that is needed
	# return dic[pair[0]],dic[pair[1]]
	return tup(dic[pair[0]]),tup(dic[pair[1]])

def tup(d):
	#reorganize a dictionary into a 3-tuple
	if d == None:
		return None
	else:
		return (d['x'],d['y'],d['z'])

def align(data_dir,key_A,key_B):
	data_files = os.listdir(data_dir)

	A = []
	B = []

	# extract the data
	for df in [data_files[0]]:
		# get the data from the file
		data = json.load(open(os.path.join(data_dir,df)))
		# find out which nodes are seen
		node_keys = sorted([key for key in data[key_A] if not data[key_A].get(key) == None and not data[key_B].get(key) == None])
		# put their coordinates in matrix A
		A.extend([tup(data[key_A][key]) for key in node_keys])
		# put the corresponding points from the other frame in matrix B
		B.extend([tup(data[key_B][key]) for key in node_keys])

	# cast the matrices to numpy arrays
	A = np.array(A)
	B = np.array(B)

	# perform the frame alignment using the Kabsch algorithm for corresponding points
	R,t = rigid_transform_3D(A,B)
	# calculate the error
	rmse = kabsch_rmse(A,B,R,t)
	return R,t,rmse

if __name__ == '__main__':
	data_dir = sys.argv[1]
	key_A = sys.argv[2]
	key_B = sys.argv[3]
	if len(sys.argv) > 4:
		output_filename = sys.argv[4]
	else:
		output_filename = None
	
	R,t,rmse = align(data_dir,key_A,key_B)

	if not output_filename == None:
		# write transformation to the output file
		data_out = {"R":np.ndarray.tolist(R),"t":np.ndarray.tolist(t),"RMSE":rmse}
		print("Writing to " + output_filename)
		json.dump(data_out,open(output_filename,'w'))
		# open(output_filename,'w').write("R:" + '\n' + '\n'.join(','.join(str(el) for el in row) for row in R))
		# open(output_filename,'a').write('\n' + "t:" + '\n' + ','.join(str(el) for el in t))
		# open(output_filename,'a').write('\n' + str(rmse))

	print('Rotation:\n',R)
	print('Translation:\n',t)
	print('RMSE:\n',rmse)

	# # make graph
	# # fig = Figure()
	# # canvas = FigureCanvas(fig)
	# # ax = fig.gca()
	# # for key in keys:
	# # 	ax.plot(distance[key],capacitance[key])
	# # 	ax.set_ylabel('Capacitance (pF)')
	# # 	ax.set_xlabel('Length (cm)')
	# # 	ax.set_title('Sensor Calibration')
	# colors = [[0,0,0],[230,159,0],[86,180,233],[0,158,115],[240,228,66],[0,114,178],[213,94,0],[204,121,167],[200,200,200]]
	# # black, orange, light blue, teal green, yellow, dark blue
	# for key in keys:
	# 	c = [n/255.0 for n in colors[int(key)]]
	# 	plt.plot(distance[key],capacitance[key],'o',color=c)
	# 	caps = np.linspace(min(capacitance[key]),max(capacitance[key]),10)
	# 	plt.plot(np.polyval(fits[int(key)],caps),caps,'-',color=c)
	# plt.ylabel('Capacitance (pF)')
	# plt.xlabel('Length (mm)')
	# plt.title('Sensor Calibration')
	# plt.show()