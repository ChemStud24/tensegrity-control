import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import json
import sys
import os
from math import sqrt
from calc_Node_Pos_cable_len_errors import *
from Tensegrity_model_inputs import inPairs_3, number_of_rods
from scipy.spatial.transform import Rotation as R

import rospy
import rosnode
import message_filters
from tensegrity.msg import SensorsStamped, ImuStamped
from phasespace.msg import Markers

from align_frames import align
from kabsch import kabsch_transformed

kabsch_file = '../calibration/dynamic1.json'

def dist(x1,x2,y1,y2,z1,z2):
    return sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

def to_centroid(points):
    centroid = np.mean(points,axis=0)
    return points - centroid

def make_plot(ax,x,y,z,mx=[],my=[],mz=[]):
    # print("Plotting!")
    ax.clear()

    # plot the nodes
    ax.scatter(x,y,z,c='black',color='black')
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("z (mm)")
    ax.scatter(mx,my,mz,c='magenta',color='magenta')

    # label the nodes
    for i in range(6):
        ax.text(x[i], y[i], z[i], str(i), color='black')

    # # label the nodes
    # for i in range(6):
    #     ax.text(mx[i], my[i], mz[i], str(i), color='magenta')

    # plot the rods and sensors as lines connecting the nodes
    from Tensegrity_model_inputs import inPairs_3, number_of_rods
    for i in range(len(inPairs_3)):
        pair = inPairs_3[i]
        # pair = [p-1 for p in pair]

        # different colors
        if i < number_of_rods:
            color = 'red' # rod
        elif i < 9:
            color = 'green' # short sensor
        else:
            color = 'blue' # long sensor

        # plot
        ax.plot([x[pair[0]],x[pair[1]]],[y[pair[0]],y[pair[1]]],[z[pair[0]],z[pair[1]]],color=color)

        # label the sensor with their measured and reconstructed lengths
        if i >= number_of_rods:
            sensor = i - number_of_rods
            measured_length = length[sensor]
            d = dist(x[pair[0]],x[pair[1]],y[pair[0]],y[pair[1]],z[pair[0]],z[pair[1]])
            print("Nodes: ",pair)
            print("Length: ",d)
            try:
                md = dist(mx[pair[0]],mx[pair[1]],my[pair[0]],my[pair[1]],mz[pair[0]],mz[pair[1]])
                print("Mocap: ",md)
            except:
                pass
            print("")
            # ax.text((x[pair[0]] + x[pair[1]])/2.0, (y[pair[0]] + y[pair[1]])/2.0, (z[pair[0]] + z[pair[1]])/2.0, str())
    plt.pause(0.05)
    # plt.show()
    # canvas.draw()


# class ReconstructionPlotter:

#     def __init__(self,ax,kabsch_file=None):
#         # self.canvas = canvas
#         self.ax = ax

#         self.strain_topic = "/strain_msg"
#         strain_sub = message_filters.Subscriber(self.strain_topic, SensorsStamped)
        
#         self.imu_topic = "/imu_msg"
#         imu_sub = message_filters.Subscriber(self.imu_topic, ImuStamped)

#         if '/phasespace_node' in rosnode.get_node_names():
#             self.mocap_topic = "/phasespace_markers"
#             mocap_sub = message_filters.Subscriber(self.mocap_topic, Markers)
#             self.kabsch_file = kabsch_file

#             # data synchronizer with mocap
#             self.time_synchornizer = message_filters.ApproximateTimeSynchronizer([strain_sub,imu_sub,mocap_sub], queue_size=10, slop=0.1)
#             self.time_synchornizer.registerCallback(self.mocap_callback)
#         else:
#             # data synchronizer without mocap
#             self.time_synchornizer = message_filters.ApproximateTimeSynchronizer([strain_sub,imu_sub], queue_size=10, slop=0.1)
#             self.time_synchornizer.registerCallback(self.no_mocap_callback)

#     def no_mocap_callback(self,strain_msg,imu_msg):
#         # calculate state reconstruction (lengths in mm) 
#         values = np.array([sensor.length for sensor in strain_msg.sensors])
#         imus = np.array([imu['x'],imu['y'],imu['z']] for imu in imu_msg.imus)
#         array = calculateNodePositions(inNodes_3,inPairs_3, values, imu=imus)

#         # plot
#         make_plot(self.ax,array[:,0],array[:,1],array[:,2])

#     def mocap_callback(self,strain_msg,imu_msg,mocap_msg):
#         # calculate state reconstruction (lengths in mm) 
#         values = np.array([sensor.length for sensor in strain_msg.sensors])
#         imus = np.array([imu['x'],imu['y'],imu['z']] for imu in imu_msg.imus)
#         array = calculateNodePositions(inNodes_3,inPairs_3, values, imu=imus)

#         # extract the motion capture data
#         mocap_coordinates = []
#         missing = []
#         for marker in mocap_msg.markers:
#             if marker.cond == -1:
#                 missing.append(marker.id)
#             else:
#                 mocap_coordinates.append([marker.x,marker.y,marker.z])
#         mocap_coordinates = np.array(mocap_coordinates)
        
#         # transform mocap coordinates to world frame
#         trans = json.load(open(self.kabsch_file))
#         R = np.array(trans["R"])
#         t = np.array(trans["t"])
#         mocap_coordinates = kabsch_transformed(mocap_coordinates,R,t)

#         # plot
#         make_plot(self.ax,array[:,0],array[:,1],array[:,2],mocap_coordinates[:,0],mocap_coordinates[:,1],mocap_coordinates[:,2])

#     def run(self, rate):
#         print("ReconstructionPlotter started! Waiting for messages.")
#         while not rospy.is_shutdown():
#             rate.sleep()

if __name__ == '__main__':

    # create the figure
    fig9 = plt.figure(figsize=(4,4))
    # canvas = FigureCanvas(fig9)
    ax = fig9.add_subplot(111, projection='3d')

    # if len(sys.argv) > 1: # plot state reconstruction from last data file in directory
    print('extracting data...')
    # get data
    data_dir = sys.argv[1]
    # pairs_file = sys.argv[2]
    data_files = sorted(os.listdir(data_dir))

    # # read pairs file
    # pairs = [line.strip().split(',') for line in open(pairs_file).readlines()]

    # initialize where we will put the data using the first data file's state reconstruction keys
    node_keys = sorted(json.load(open(os.path.join(data_dir,data_files[0])))['state reconstruction'].keys())
    # y = {key:[] for key in node_keys}
    # x = {key:[] for key in node_keys}
    # z = {key:[] for key in node_keys}

    # initialize where we will put the data using the first data file's sensor keys
    sensor_keys = sorted(json.load(open(os.path.join(data_dir,data_files[0])))['sensors'].keys())
    # capacitance = {key:[] for key in sensor_keys}
    # length = {key:[] for key in sensor_keys}

    imu_keys = ['yaw','pitch','roll']
    # yaw = []
    # pitch = []
    # roll = []
    # acc_x = []
    # acc_y = []
    # acc_z = []
    # time_all = []

    # count = 0
    # count_pre = 0

    # extract the data
    for df in data_files:
        print(df)
        data = json.load(open(os.path.join(data_dir,df)))

        x = []
        y = []
        z = []
        #mocap
        mx = []
        my = []
        mz = []
        length = []
        rotation = []

        for key in sorted(data['sensors'].keys()):
            length.append(float(data['sensors'][key]['length']))

        # recalculate state reconstruction if prompted
        if len(sys.argv) > 2:
            print('recalculating state reconstruction...')
            # calculate state reconstruction lengths in cm 
            values = np.array(length)
            # print(values)
            # HARD CODE ERASE LATER
            # values = np.array([164,170,165,175,175,169,309,305,310])
            # values = np.array([167,171,173,173,180,155,310,309,305])
            # HARD CODE ERASE LATER
            array = calculateNodePositions(inNodes_3,inPairs_3, values,angle0 = [-198,265,18])
            data['state reconstruction'] = {str(node):{'x':xyz[0],'y':xyz[1],'z':xyz[2]} for node,xyz in enumerate(array)}

        for key in node_keys:
            x.append(float(data['state reconstruction'][key]['x']))
            y.append(float(data['state reconstruction'][key]['y']))
            z.append(float(data['state reconstruction'][key]['z']))
            if not data.get('mocap').get(key) == None:
                mx.append(float(data['mocap'][key]['x']))
                my.append(float(data['mocap'][key]['y']))
                mz.append(float(data['mocap'][key]['z']))

        # translate to centroid
        r_points = np.vstack((np.array([x]),np.array([y]),np.array([z]))).T
        r_points = to_centroid(r_points)
        x = r_points[:,0]
        y = r_points[:,1]
        z = r_points[:,2]

        if len(mx) > 0:
            # apply the rotation from Kabsch
            from_kabsch = json.load(open(kabsch_file))
            R = from_kabsch.get('R')
            t = from_kabsch.get('t')
            m_points = np.vstack((np.array([mx]),np.array([my]),np.array([mz]))).T
            m_points = kabsch_transformed(m_points,R,t)
            m_points = to_centroid(m_points) # translate to centroid
            mx = m_points[:,0]
            my = m_points[:,1]
            mz = m_points[:,2]

        make_plot(ax,x,y,z,mx,my,mz)

    plt.show()
            # for key in imu_keys:
            #     rotation.append(float(data['imu'][key]))

            # position_temp = []
            # target_temp = []
            # speed_temp = []
            # yaw.append(float(data['imu']['yaw']))
            # pitch.append(float(data['imu']['pitch']))
            # roll.append(float(data['imu']['roll']))
            # acc_x.append(float(data['imu']['ax']))
            # acc_y.append(float(data['imu']['ay']))
            # acc_z.append(float(data['imu']['az']))
            # time_all.append(float(data['header']['secs']))
    # else:
    #     # read live data from ROS
    #     rospy.init_node('reconstruction_plotter')
    #     rate = rospy.Rate(30)
    #     plotter = ReconstructionPlotter(ax,kabsch_file)
    #     plotter.run(rate)

    # # get points as array
    # all_points = np.concatenate((np.array([x]),np.array([y]),np.array([z])),axis=0).T

    # # incorporate the IMU data
    # imu_location = np.array([[(x[0] + x[1])/2.0, (y[0] + y[1])/2.0, (z[0] + z[1])/2.0]])
    # all_points = all_points - imu_location
    # print("Translated: ",all_points)

    # # imu rotation
    # r = R.from_euler('xyz',rotation,degrees=True)
    # all_points = r.apply(all_points)
    # print("Rotated: ",all_points)

    # # reformat for plotting
    # x = all_points[:,0]
    # y = all_points[:,1]
    # z = all_points[:,2]

    """
    # apply the rotation from Kabsch
    # R = np.array([[0.32805444,-0.36761773,-0.87019394],
    #  [-0.20920188,-0.9265721,0.31256794],
    #  [-0.92120294,0.0795069,-0.38087241]])
    # t = np.array([142.64448869,992.89540288,12.34203898])
    # R = np.array([[-0.52323422,-0.32128183,-0.78930598],
    #  [-0.65480777,0.74433897 ,0.13109649],
    #  [0.54539228,0.58543786,-0.59984146]])
    # t = np.array([87.63360153,857.3362514,372.38551799])
    from_kabsch = json.load(open('../calibration/alignment.json'))
    R = from_kabsch.get('R')
    t = from_kabsch.get('t')
    all_points = np.vstack((np.array([x]),np.array([y]),np.array([z]))).T
    all_points = kabsch_transformed(all_points,R,t)
    x = all_points[:,0]
    y = all_points[:,1]
    z = all_points[:,2]

    # plot the nodes
    ax.scatter(x,y,z,c='black',color='black')
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("z (mm)")
    ax.scatter(mx,my,mz,c='magenta',color='magenta')

    # label the nodes
    for i in range(6):
        ax.text(x[i], y[i], z[i], str(i), color='black')

    # label the nodes
    for i in range(6):
        ax.text(mx[i], my[i], mz[i], str(i), color='magenta')

    # plot the rods and sensors as lines connecting the nodes
    for i in range(len(inPairs_3)):
        pair = inPairs_3[i]
        # pair = [p-1 for p in pair]

        # different colors
        if i < number_of_rods:
            color = 'red' # rod
        elif i < 9:
            color = 'green' # short sensor
        else:
            color = 'blue' # long sensor

        # plot
        ax.plot([x[pair[0]],x[pair[1]]],[y[pair[0]],y[pair[1]]],[z[pair[0]],z[pair[1]]],color=color)

        # label the sensor with their measured and reconstructed lengths
        if i >= number_of_rods:
            sensor = i - number_of_rods
            measured_length = length[sensor]
            d = dist(x[pair[0]],x[pair[1]],y[pair[0]],y[pair[1]],z[pair[0]],z[pair[1]])
            ax.text((x[pair[0]] + x[pair[1]])/2.0, (y[pair[0]] + y[pair[1]])/2.0, (z[pair[0]] + z[pair[1]])/2.0, str())

    plt.show()
    """