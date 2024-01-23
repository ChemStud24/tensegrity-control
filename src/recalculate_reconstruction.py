import numpy as np
import json
import sys
import os
from math import sqrt
from calc_Node_Pos_cable_len_errors import *

def dist(x1,x2,y1,y2,z1,z2):
    return sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

# create the figure
fig9 = plt.figure(figsize=(4,4))

# recalculate state reconstruction for each data file in directory
print('extracting data...')
# get data
data_dir = sys.argv[1]
data_files = os.listdir(data_dir)

# initialize where we will put the data using the first data file's state reconstruction keys
node_keys = sorted(json.load(open(os.path.join(data_dir,data_files[0])))['state reconstruction'].keys())

# initialize where we will put the data using the first data file's sensor keys
sensor_keys = sorted(json.load(open(os.path.join(data_dir,data_files[0])))['sensors'].keys())

# extract the data
for df in data_files:
    data = json.load(open(os.path.join(data_dir,df)))

    length = []
    # rotation = []

    for key in sorted(data['sensors'].keys()):
        length.append(float(data['sensors'][key]['length']))

    # recalculate state reconstruction if prompted
    print('recalculating state reconstruction...')
    # calculate state reconstruction lengths in cm 
    values = np.array(length)
    array = calculateNodePositions(inNodes_3,inPairs_3, values,angle0 = [-198,265,18])
    data['state reconstruction'] = {str(node):{'x':xyz[0],'y':xyz[1],'z':xyz[2]} for node,xyz in enumerate(array)}
    json.dump(data,open(os.path.join(data_dir,df),'w'))