#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Vector3
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import math

class ECCameraModel():

    def update(self):
        self.hscale = self.width / (2 * math.pi)
        self.vscale = self.height / self.vfov

    def setInfo(self, data):
        dims = data.points.layout.dim

        if len(dims ==3):
            self.height = dims[1].size
            self.width = dims[2].size

            self.vfov = data.fov_v
        else:
            rospy.logerror("Error in info!")

    def projectPixelTo3dRay(self, uv):
        uv = np.asarray(uv)
        uv = uv.reshape(-1,2)
        ray = np.ndarray(shape=(uv.shape[0], 3), dtype=float)

        x = uv[:,0] #?
        y = uv[:,1] #?

        theta = (x - (self.width / 2)) / self.hscale

        ray[:,0] = np.sin(theta);
        ray[:,2] = np.cos(theta);

        ray[:, 1] = (y - (self.height / 2)) / self.vscale

        ##The above codes already ensures that x^2 + z^2 =1
        #ranges = ray[:,0] * ray[:,0] + ray[:,2] * ray[:,2]
        #ray /= ranges

        return ray

    def projectPixelsTo3d(self, uv, image):
        uv = np.asarray(uv)
        uv = uv.reshape(-1, 2)

        ray = self.projectPixelTo3dRay(uv)

        coord = (uv[:,1], uv[:0])
        depth = image[coord]

        points = ray * depth[:,None]

        return points

    def __init__(self):
        self.height = 480
        self.width = 960
        self.vfov = math.pi
        self.update()


if __name__ == '__main__':
    try:
        model = ECCameraModel()
        pass
        pass
        pass
        model.projectPixelTo3dRay([[40,72],[100,90]])

        image = np.random.random((480,640))

        model.projectPixelsTo3d([[40,72],[100,90]], image)


    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
