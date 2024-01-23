import matplotlib.pyplot as plt
import rospy
import numpy as np
from matplotlib.animation import FuncAnimation

from tensegrity.msg import ImuStamped
from Tensegrity_model_inputs import inPairs_3, number_of_rods, L


class Visualiser:
    def __init__(self):
    	# figure and axes
    	self.fig = plt.figure(figsize=(4,4))
    	self.ax = [self.fig.add_subplot(311),self.fig.add_subplot(312),self.fig.add_subplot(313)]
        self.ln = [self.ax[i].plot([[0]],[[0,0,0]]) for i in range(len(self.ax))]
        # self.ln = self.ax[0].plot([[0]],[[0,0,0]])
    	# reconstruction and mocap scatters
        # self.scat = [self.ax[i].scatter([],[]) for i in range(len(self.ax))]
        # self.scat = True
        # data
        self.adata = [[0,0,0,0]]
        self.gdata = [[0,0,0,0]]
        self.mdata = [[0,0,0,0]]

    def plot_init(self):
        # self.ax.set_xlim(-2*L, 2*L)
        # self.ax.set_ylim(-2*L, 2*L)
        # self.ax.set_zlim(-2*L, 2*L)
        self.ax[0].set_xlabel("Time (s)")
        self.ax[0].set_ylabel("Acceleration (g)")
        self.ax[0].set_xlim(0,120)
        self.ax[0].set_ylim(-2,2)
        self.ax[1].set_xlabel("Time (s)")
        self.ax[1].set_ylabel("Angular Velocity (rad/s)")
        self.ax[1].set_xlim(0,120)
        self.ax[1].set_ylim(-100,100)
        self.ax[2].set_xlabel("Time (s)")
        self.ax[2].set_ylabel("Magnetic Field (mT)")
        self.ax[2].set_xlim(0,120)
        self.ax[2].set_ylim(-50,50)

        return self.ln

    def callback(self, msg):
    	# reconstruction
        self.adata.append([msg.header.stamp.secs,msg.imus[0].ax,msg.imus[0].ay,msg.imus[0].az])
        self.gdata.append([msg.header.stamp.secs,msg.imus[0].gx,msg.imus[0].gy,msg.imus[0].gz])
        self.mdata.append([msg.header.stamp.secs,msg.imus[0].mx,msg.imus[0].my,msg.imus[0].mz])
        print('got some data')

    def update_plot(self, frame):
        print('lets update')
        adata = np.array(self.adata)
        for i in range(3):
            self.ln[0][i].set_data(adata[:,0]-adata[1,0],adata[:,i+1])
        print(adata[:,1] - adata[1,0])
        # self.ax[0].plot(adata[:,0],adata[:,1:])
        gdata = np.array(self.gdata)
        for i in range(3):
            self.ln[1][i].set_data(gdata[:,0]-gdata[1,0],gdata[:,i+1])
        # self.ax[1].plot(gdata[:,0],gdata[:,1:])
        mdata = np.array(self.mdata)
        for i in range(3):
            self.ln[2][i].set_data(mdata[:,0]-mdata[1,0],mdata[:,i+1])
        # self.ax[2].plot(mdata[:,0],mdata[:,1:])
        print('updated the plot')
        
        return self.ln


rospy.init_node('imu_plotter')
vis = Visualiser()
sub = rospy.Subscriber('/imu_msg', ImuStamped, vis.callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
print('i got here')
plt.show(block=True) 