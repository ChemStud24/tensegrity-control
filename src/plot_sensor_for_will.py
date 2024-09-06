import numpy as np
from math import sqrt
import sys
import os
import json
import matplotlib.pyplot as plt
import csv
from natsort import natsorted 
from vpython import *
# from matplotlib.figure import Figure

if __name__ == '__main__':
    data_dir = sys.argv[1]
    #output_filename = sys.argv[3]
    data_files = os.listdir(data_dir)
    data_files = natsorted(data_files)

    # initialize where we will put the data using the first data file's sensor keys

    time = []
    sr_keys = sorted(json.load(open(os.path.join(data_dir,data_files[0])))['state_reconstruction'].keys())
    y_sr = {key:[] for key in sr_keys}
    x_sr = {key:[] for key in sr_keys}
    z_sr = {key:[] for key in sr_keys}
    count = 0
    count_pre = 0
    scene = canvas(width=1280,height=800)
    scene.autoscale = False
    scene.center = vec(0,-2.5,0)
    scene.camera.pos = vector(9,10,0)
    scene.camera.axis = vector(0,-5,0)
    # extract the data
    for df in data_files:
            data = json.load(open(os.path.join(data_dir,df)))
            x_temp = []
            y_temp = []
            z_temp = []
            x_sr_temp = []
            y_sr_temp = []
            z_sr_temp = []
            for key in data['state_reconstruction'].keys():
                if data['state_reconstruction'][key] is None:
                    break
                else:
                    x_sr_temp.append(float(data['state_reconstruction'][key]['x']))
                    y_sr_temp.append(float(data['state_reconstruction'][key]['y']))
                    z_sr_temp.append(float(data['state_reconstruction'][key]['z']))
                if key == "5" and len(x_sr_temp) == 6:
                    for i in range(6):
                        y_sr[str(i)].append(y_sr_temp[i])
                        x_sr[str(i)].append(x_sr_temp[i])
                        z_sr[str(i)].append(z_sr_temp[i])
                    time.append(float(data['header']['secs']))

    colors = [[255,0,0],[0,255,0],[0,0,255],[0,158,115],[240,228,66],[0,114,178],[213,94,0],[204,121,167],[200,200,200]]
    node_radius = 0.2
    bar_radius = 0.05
    cab_radius = 0.05
    nodes1_sr = []
    nodes2_sr = []
    nodes3_sr = []
    nodes4_sr = []
    nodes5_sr = []
    nodes6_sr = []
    r_x = 0
    r_y = pi/2
    r_z = 0
    length_sr = len(x_sr["0"])

    node0_sr = sphere(pos = vec(x_sr["0"][0]/100, y_sr["0"][0]/100, z_sr["0"][0]/100), radius = node_radius, color = vec(colors[0][0]/255, colors[0][1]/255, colors[0][2]/255))
    node1_sr = sphere(pos = vec(x_sr["1"][0]/100, y_sr["1"][0]/100, z_sr["1"][0]/100), radius = node_radius, color = node0_sr.color)
    node2_sr = sphere(pos = vec(x_sr["2"][0]/100, y_sr["2"][0]/100, z_sr["2"][0]/100), radius = node_radius, color = vec(colors[1][0]/255, colors[1][1]/255, colors[1][2]/255))
    node3_sr = sphere(pos = vec(x_sr["3"][0]/100, y_sr["3"][0]/100, z_sr["3"][0]/100), radius = node_radius, color = node2_sr.color)
    node4_sr = sphere(pos = vec(x_sr["4"][0]/100, y_sr["4"][0]/100, z_sr["4"][0]/100), radius = node_radius, color = vec(colors[2][0]/255, colors[2][1]/255, colors[2][2]/255))
    node5_sr = sphere(pos = vec(x_sr["5"][0]/100, y_sr["5"][0]/100, z_sr["5"][0]/100), radius = node_radius, color = node4_sr.color)
    com_sr = (node0_sr.pos + node1_sr.pos + node2_sr.pos + node3_sr.pos + node4_sr.pos + node5_sr.pos) / 6 + vec(-1,5,0)
    node0_sr.pos = node0_sr.pos - com_sr
    node1_sr.pos = node1_sr.pos - com_sr
    node2_sr.pos = node2_sr.pos - com_sr
    node3_sr.pos = node3_sr.pos - com_sr
    node4_sr.pos = node4_sr.pos - com_sr
    node5_sr.pos = node5_sr.pos - com_sr
    bar0_sr = cylinder(pos = node0_sr.pos, axis = node1_sr.pos-node0_sr.pos, radius = bar_radius, color = vec(1, 1, 1))
    bar1_sr = cylinder(pos = node2_sr.pos, axis = node3_sr.pos-node2_sr.pos, radius = bar_radius, color = bar0_sr.color)
    bar2_sr = cylinder(pos = node4_sr.pos, axis = node5_sr.pos-node4_sr.pos, radius = bar_radius, color = bar0_sr.color)
    cab0_sr = cylinder(pos = node3_sr.pos, axis = node5_sr.pos-node3_sr.pos, radius = cab_radius, color = vec(0, 1, 1))
    cab1_sr = cylinder(pos = node1_sr.pos, axis = node3_sr.pos-node1_sr.pos, radius = cab_radius, color = cab0_sr.color)
    cab2_sr = cylinder(pos = node1_sr.pos, axis = node5_sr.pos-node1_sr.pos, radius = cab_radius, color = cab0_sr.color)
    cab3_sr = cylinder(pos = node0_sr.pos, axis = node2_sr.pos-node0_sr.pos, radius = cab_radius, color = cab0_sr.color)
    cab4_sr = cylinder(pos = node0_sr.pos, axis = node4_sr.pos-node0_sr.pos, radius = cab_radius, color = cab0_sr.color)
    cab5_sr = cylinder(pos = node2_sr.pos, axis = node4_sr.pos-node2_sr.pos, radius = cab_radius, color = cab0_sr.color)
    cab6_sr = cylinder(pos = node2_sr.pos, axis = node5_sr.pos-node2_sr.pos, radius = cab_radius, color = cab0_sr.color)
    cab7_sr = cylinder(pos = node0_sr.pos, axis = node3_sr.pos-node0_sr.pos, radius = cab_radius, color = cab0_sr.color)
    cab8_sr = cylinder(pos = node1_sr.pos, axis = node4_sr.pos-node1_sr.pos, radius = cab_radius, color = cab0_sr.color)




    for i in range(0,length_sr):
        node0_sr.pos = vec(x_sr["0"][i]/100, y_sr["0"][i]/100, z_sr["0"][i]/100)
        node1_sr.pos = vec(x_sr["1"][i]/100, y_sr["1"][i]/100, z_sr["1"][i]/100)
        node2_sr.pos = vec(x_sr["2"][i]/100, y_sr["2"][i]/100, z_sr["2"][i]/100)
        node3_sr.pos = vec(x_sr["3"][i]/100, y_sr["3"][i]/100, z_sr["3"][i]/100)
        node4_sr.pos = vec(x_sr["4"][i]/100, y_sr["4"][i]/100, z_sr["4"][i]/100)
        node5_sr.pos = vec(x_sr["5"][i]/100, y_sr["5"][i]/100, z_sr["5"][i]/100)
        com_sr = (node0_sr.pos + node1_sr.pos + node2_sr.pos + node3_sr.pos + node4_sr.pos + node5_sr.pos) / 6 + vec(-1,5,0)
        node0_sr.pos = node0_sr.pos - com_sr
        node1_sr.pos = node1_sr.pos - com_sr
        node2_sr.pos = node2_sr.pos - com_sr
        node3_sr.pos = node3_sr.pos - com_sr
        node4_sr.pos = node4_sr.pos - com_sr
        node5_sr.pos = node5_sr.pos - com_sr
        bar0_sr.pos = node0_sr.pos
        bar0_sr.axis = node1_sr.pos-node0_sr.pos
        bar1_sr.pos = node2_sr.pos
        bar1_sr.axis = node3_sr.pos-node2_sr.pos
        bar2_sr.pos = node4_sr.pos
        bar2_sr.axis = node5_sr.pos-node4_sr.pos
        cab0_sr.pos = node3_sr.pos
        cab0_sr.axis = node5_sr.pos-node3_sr.pos
        cab1_sr.pos = node1_sr.pos
        cab1_sr.axis = node3_sr.pos-node1_sr.pos
        cab2_sr.pos = node1_sr.pos
        cab2_sr.axis = node5_sr.pos-node1_sr.pos
        cab3_sr.pos = node0_sr.pos
        cab3_sr.axis = node2_sr.pos-node0_sr.pos
        cab4_sr.pos = node0_sr.pos
        cab4_sr.axis = node4_sr.pos-node0_sr.pos
        cab5_sr.pos = node2_sr.pos
        cab5_sr.axis = node4_sr.pos-node2_sr.pos
        cab6_sr.pos = node2_sr.pos
        cab6_sr.axis = node5_sr.pos-node2_sr.pos
        cab7_sr.pos = node0_sr.pos
        cab7_sr.axis = node3_sr.pos-node0_sr.pos
        cab8_sr.pos = node1_sr.pos
        cab8_sr.axis = node4_sr.pos-node1_sr.pos
        #f.write(str(x["0"][i]/100) + ',' + str(y["0"][i]/100) + ',' + str(z["0"][i]/100,) + ',' + '\n')
        rate(10)

    
