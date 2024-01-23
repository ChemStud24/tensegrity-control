#!/usr/bin/python

# manipulating and writing data
import os
import numpy as np
import shutil
import sys
import json
from cv_bridge import CvBridge
import cv2

# ROS Messages
import rospy
import message_filters
from sensor_msgs.msg import Image
# from geometry_msgs.msg import QuaternionStamped
# from tensegrity.msg import MotorsStamped, SensorsStamped, ImuStamped, NodesStamped
from tensegrity.msg import TensegrityStamped, NodesStamped

# shape reconstruction
# from calc_Node_Pos_cable_len_errors import *
# from Tensegrity_model_inputs import *

# #initial values for the node positions
# inNodes_3 = np.array([[0.000, 0.000, 0.0000], [-113.00, 0.000, -219.00], [-56.00, 130.00, 0.000], [5.00, 10.00, -200.00], 
# [-100.00, 0.000, 0.000], [-50.00, 100.00, -200.00]])

class SyncDataWriter:

    def __init__(self, output_dir='output'):
        self.bridge = CvBridge()
        self.depth_scale = 4000  # L515

        self.depth_topic = "/depth_images"
        self.color_topic = "/rgb_images"
        self.control_topic = "/control_msg"
        self.reconstruction_topic = "/reconstruction_msg"
        color_im_sub = message_filters.Subscriber(self.color_topic, Image)
        depth_im_sub = message_filters.Subscriber(self.depth_topic, Image)
        control_sub = message_filters.Subscriber(self.control_topic, TensegrityStamped)
        reconstruction_sub = message_filters.Subscriber(self.reconstruction_topic,NodesStamped)

        # time synchronizer
        self.time_synchornizer = message_filters.ApproximateTimeSynchronizer([color_im_sub,depth_im_sub,control_sub,reconstruction_sub], queue_size=10, slop=0.1)
        self.time_synchornizer.registerCallback(self.callback)

        # output dirs
        self.color_dir = os.path.join(output_dir, 'color')
        self.depth_dir = os.path.join(output_dir, 'depth')
        self.stamp_dir = os.path.join(output_dir, 'data')
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
        os.mkdir(output_dir)
        os.mkdir(self.color_dir)
        os.mkdir(self.depth_dir)
        os.mkdir(self.stamp_dir)

        self.count = 0

    def callback(self, color_msg, depth_msg, control_msg, reconstruction_msg):
        # global inNodes_3
        print("Received synchronized data", self.count)

        color_im = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8') # double check this
        depth_im = self.bridge.imgmsg_to_cv2(depth_msg, 'mono16')
        cv2.imwrite(os.path.join(self.color_dir, str(self.count).zfill(4) + ".png"), color_im)
        cv2.imwrite(os.path.join(self.depth_dir, str(self.count).zfill(4) + ".png"), depth_im)

        # data = {}
        # data['header'] = {'seq':color_msg.header.seq,'secs':color_msg.header.stamp.to_sec()}
        
        # # calculate state reconstruction (lengths in mm) 
        # values = np.array([sensor.length for sensor in strain_msg.sensors])
        # imus = np.array([[imu.x,imu.y,imu.z] for imu in imu_msg.imus])
        # array = calculateNodePositions(self.inNodes_3,inPairs_3, values,imus,self.angle0)
        
        # format syncronized data
        data = {}
        data['header'] = {'seq':control_msg.header.seq,'secs':control_msg.header.stamp.to_sec()}
        data['info'] = {'min_length':control_msg.info.min_length,'RANGE':control_msg.info.RANGE,'RANGE024':control_msg.info.RANGE024,'RANGE135':control_msg.info.RANGE135,'MAX_RANGE':control_msg.info.MAX_RANGE,'MIN_RANGE':control_msg.info.MIN_RANGE,'max_speed':control_msg.info.max_speed,'tol':control_msg.info.tol,'low_tol':control_msg.info.low_tol,'P':control_msg.info.P,'I':control_msg.info.I,'D':control_msg.info.D}
        data['motors'] = {}
        for motor in control_msg.motors:
            data['motors'][motor.id] = {'target':motor.target,'position':motor.position,'speed':motor.speed,'done':motor.done}
        data['sensors'] = {}
        for sensor in control_msg.sensors:
            data['sensors'][sensor.id] = {'length':sensor.length,'capacitance':sensor.capacitance}
        data['imu'] = {}
        for imu in control_msg.imus:
            # data['imu'][imu.id] = {'ax':imu.ax,'ay':imu.ay,'az':imu.az,'gx':imu.gx,'gy':imu.gy,'gz':imu.gz,'mx':imu.mx,'my':imu.my,'mz':imu.mz}
            data['imu'][imu.id] = {'x':imu.x,'y':imu.y,'z':imu.z}#,'q1':imu.q1,'q2':imu.q2,'q3':imu.q3,'q4':imu.q4}#,'yaw':imu_msg.yaw, 'pitch':imu_msg.pitch,'roll':imu_msg.roll}
        # data['mocap'] = {}
        # for marker in mocap_msg.markers:
        #     if marker.cond == -1:
        #         data['mocap'][marker.id] = {'cond':False,'x':None,'y':None,'z':None}
        #     else:
        #         data['mocap'][marker.id] = {'x':marker.x,'y':marker.y,'z':marker.z,'cond':True}
        # data['state reconstruction'] = {node:{'x':xyz[0],'y':xyz[1],'z':xyz[2]} for node,xyz in enumerate(array)}

        # maybe try this later
        # data['state_reconstruction'] = {}
        # data['tracking'] = {}
        # for node in control_msg.nodes:
        #     data['tracking'][node.id] = {'x':node.x,'y':node.y,'z':node.z}
        
        # COM, principal_axis, endcaps = get_pose()
        # # data['tracking'] = {'COM_x':COM.tolist()[0],'COM_y':COM.tolist()[1],'principal_axis':principal_axis.tolist()}
        # data['tracking'] = {node:{'x':xyz[0],'y':xyz[1],'z':xyz[2]} for node,xyz in enumerate(endcaps)}
        
        # write data to file
        json.dump(data,open(os.path.join(self.stamp_dir, str(self.count).zfill(4) + ".json"),'w'))

        # self.inNodes_3 = np.array(array)

        self.count += 1

    def run(self, rate):
        print("SyncDataWriter started! Waiting for messages.")
        while not rospy.is_shutdown():
            rate.sleep()
            # pass

if __name__ == '__main__':

    rospy.init_node('synchronizer')
    rate = rospy.Rate(30)
    if len(sys.argv) > 1:
        writer = SyncDataWriter(output_dir=sys.argv[1])
    else:
        writer = SyncDataWriter(output_dir='../data/output')
    writer.run(rate)