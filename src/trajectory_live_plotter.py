import matplotlib.pyplot as plt
import rospy
import numpy as np
from matplotlib.animation import FuncAnimation

from tensegrity.msg import TensegrityStamped

MAX_NUM_POINTS = 10

class Visualiser:
    def __init__(self):
        # figure and axes
        self.fig = plt.figure(figsize=(5,5),num='Real-time Plan')
        # self.fig = plt.figure(num='Real-time Plan')
        self.ax = self.fig.add_subplot(111)
        # data
        self.trajectory = np.array([[0,0]])
        self.COMs = np.array([[0,0]])
        self.PAs = np.array([[1,0]])
        self.actions = ['Next: act1','Then: act2'] # list of strings

        self.trajectory_plot = self.ax.plot([],[],'go')[0]
        self.com_plot = self.ax.plot([],[],'ro')[0]
        self.lines = [self.ax.plot([],[],'m-')[0] for l in range(MAX_NUM_POINTS)]
        self.unit_vector_length = 0.1

        self.tips = self.COMs + self.PAs * self.unit_vector_length

        self.labels = [self.ax.text(1.9,1.55 + 0.2*i,act,ha='left',va='center',fontsize=15,color='Red') for i,act in enumerate(self.actions)]

    def plot_init(self):
        self.ax.axis('equal')
        # self.ax.set_xlim(-0.1, 2)
        # self.ax.set_ylim(-0.1, 2)
        self.ax.set_xlim(2, -0.1)
        self.ax.set_ylim(2, -0.1)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        return self.trajectory

    def callback(self, msg):
    	# trajectory
        self.trajectory = np.array([[point.x,point.y] for point in msg.trajectory.trajectory])
        # centers of mass
        self.COMs = np.array([[com.x,com.y] for com in msg.trajectory.COMs])
        # principal axes
        self.PAs = np.array([[PA.x,PA.y] for PA in msg.trajectory.PAs])
        # tips
        self.tips = self.COMs + self.PAs * self.unit_vector_length
        # actions
        if len(msg.actions) > 0:
            self.actions = ['Next: ' + msg.actions[0]] + ['Then: ' + act for act in msg.actions[1:]]

    def update_plot(self, frame):
        # print(self.trajectory)
        if len(self.trajectory) > 0:
            self.trajectory_plot.set_data(self.trajectory[:,0],self.trajectory[:,1])
            self.com_plot.set_data(self.COMs[:,0],self.COMs[:,1])
            for i in range(len(self.tips)):
                com = self.COMs[i]
                tip = self.tips[i]
                lin = self.lines[i]
                lin.set_data(np.array([com[0],tip[0]]),np.array([com[1],tip[1]]))

            for i in range(len(self.labels)):
                self.labels[i].set_text(self.actions[i])

        return self.trajectory


rospy.init_node('trajectory_plotter')
vis = Visualiser()
sub = rospy.Subscriber('/control_msg', TensegrityStamped, vis.callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 