import numpy as np
from math import sqrt
import sys
import os
import json
import matplotlib.pyplot as plt
# from matplotlib.figure import Figure

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

if __name__ == '__main__':
	data_dir = sys.argv[1]
	pairs_file = sys.argv[2]
	output_filename = sys.argv[3]
	data_files = os.listdir(data_dir)

	# read pairs file
	pairs = [line.strip().split(',') for line in open(pairs_file).readlines()]

	# initialize where we will put the data using the first data file's sensor keys
	keys = sorted(json.load(open(os.path.join(data_dir,data_files[0])))['sensors'].keys())
	capacitance = {key:[] for key in keys}
	distance = {key:[] for key in keys}

	# extract the data
	for df in data_files:
		data = json.load(open(os.path.join(data_dir,df)))
		for key in data['sensors'].keys():
			points = get_points(data['mocap'],pairs[int(key)])
			if not None in points and data['sensors'][key]['capacitance'] > 0:
				capacitance[key].append(float(data['sensors'][key]['capacitance']))
				distance[key].append(dist(points))
				print(key,dist(points))

	# perform the linear fits
	fits = [np.polyfit(capacitance[key],distance[key],1) for key in keys]

	# transform into m and b for linear fits
	m = np.array([fit[0] for fit in fits])
	b = np.array([fit[1] for fit in fits])
	m2 = 1.0/m
	b2 = -b/m
	m = m2
	b = b2

	# write calibration to the JSON output file
	data = {'m':np.ndarray.tolist(m),'b':np.ndarray.tolist(b)}
	json.dump(data,open(output_filename,'w'))

	# # write calibration to the output file
	# open(output_filename,'w').write('\n'.join(','.join(str(coefficient) for coefficient in fit) for fit in fits))

	# make graph
	# fig = Figure()
	# canvas = FigureCanvas(fig)
	# ax = fig.gca()
	# for key in keys:
	# 	ax.plot(distance[key],capacitance[key])
	# 	ax.set_ylabel('Capacitance (pF)')
	# 	ax.set_xlabel('Length (cm)')
	# 	ax.set_title('Sensor Calibration')
	colors = [[0,0,0],[230,159,0],[86,180,233],[0,158,115],[240,228,66],[0,114,178],[213,94,0],[204,121,167],[200,200,200]]
	# black, orange, light blue, teal green, yellow, dark blue
	for key in keys:
		c = [n/255.0 for n in colors[int(key)]]
		plt.plot(distance[key],capacitance[key],'o',color=c)
		caps = np.linspace(min(capacitance[key]),max(capacitance[key]),10)
		plt.plot(np.polyval(fits[int(key)],caps),caps,'-',color=c)
	plt.ylabel('Capacitance (pF)')
	plt.xlabel('Length (mm)')
	plt.title('Sensor Calibration')
	plt.show()