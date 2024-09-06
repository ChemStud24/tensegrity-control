#!/usr/bin/python

from  __future__ import print_function

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
from tensegrity.msg import MotorsStamped, SensorsStamped, ImuStamped, NodesStamped

# other
import argparse

# tracking
import rosgraph
import cv_bridge
from std_msgs.msg import Float64MultiArray
from tensegrity_perception.srv import InitTracker, InitTrackerRequest, InitTrackerResponse
from tensegrity_perception.srv import GetPose, GetPoseRequest, GetPoseResponse
from sensor_msgs.msg import Image
from run_tensegrity_tracking import quat2vec

class TrackingVisualizer:

    def __init__(self, output_dir='output'):
        self.bridge = CvBridge()
        self.depth_scale = 1000  # D435

        self.depth_topic = "/depth_images"
        self.color_topic = "/rgb_images"
        self.control_topic = "/control_msg"
        self.strain_topic = "/strain_msg"
        self.imu_topic = "/imu_msg"
        color_im_sub = message_filters.Subscriber(self.color_topic, Image)
        depth_im_sub = message_filters.Subscriber(self.depth_topic, Image)
        control_sub = message_filters.Subscriber(self.control_topic, MotorsStamped)
        strain_sub = message_filters.Subscriber(self.strain_topic, SensorsStamped)
        imu_sub = message_filters.Subscriber(self.imu_topic, ImuStamped)

        # time synchronizer
        self.time_synchornizer = message_filters.ApproximateTimeSynchronizer([color_im_sub,control_sub,strain_sub], queue_size=30, slop=0.1)
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

    def callback(self, color_msg, control_msg, strain_msg):
        print("Received synchronized data", self.count)
        COM, principal_axis = get_pose()

        color_im = self.bridge.imgmsg_to_cv2(color_msg, 'rgb8')

        cv2.imshow('RealSense', images)
                key = cv2.waitKey(1)
                if (key & 0xFF) == ord('q'):
                    break

        self.count += 1

    def run(self, rate):
        print("TrackingVisualizer started! Waiting for messages.")
        while not rospy.is_shutdown():
            # rate.sleep()
            pass

def get_pose():
    service_name = "get_pose"
    rospy.loginfo(f"Waiting for {service_name} service...")
    rospy.wait_for_service(service_name)
    rospy.loginfo(f"Found {service_name} service.")
    # poses = []
    vectors = []
    centers = []
    try:
        request = GetPoseRequest()
        get_pose_srv = rospy.ServiceProxy(service_name, GetPose)
        rospy.loginfo("Request sent. Waiting for response...")
        response: GetPoseResponse = get_pose_srv(request)
        rospy.loginfo(f"Got response. Request success: {response.success}")
        if response.success:
            for pose in response.poses:
                vector = quat2vec([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
                unit_vector = vector/np.linalg.norm(vector)
                center = [pose.position.x,pose.position.y,pose.position.z]
                # T = np.eye(4)
                # T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
                # T[:3, :3] = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                #                                 pose.orientation.w]).as_matrix()
                # poses.append(T)
                centers.append(center)
                vectors.append(unit_vector.tolist())
        COM = np.mean(np.array(centers),axis=0)
        principal_axis = np.mean(np.array(vectors),axis=0)
    except rospy.ServiceException as e:
        rospy.loginfo(f"Service call failed: {e}")
    return COM, principal_axis


if __name__ == '__main__':

    rospy.init_node('tracking_visualizer')
    rate = rospy.Rate(30)
    if len(sys.argv) > 1:
        viz = TrackingVisualizer()
    viz.run(rate)