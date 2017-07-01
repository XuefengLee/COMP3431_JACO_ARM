import numpy as np
import cv2
import rospy
import rospkg
import sys
import struct
import ctypes
import math
from sympy import *
import tf
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from PIL import Image as pilImage

import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from copy import deepcopy
import Tkinter as tki
from PIL import ImageTk

class calibration:
    points = []
    pointsRef = []
    # cameraPos
    # cameraOrient
    done = False

    def calPos(self):
        x = Symbol('x')
        y = Symbol('y')
        z = Symbol('z')
        x01 = self.pointsRef[0][0]
        y01 = self.pointsRef[0][1]
        z01 = self.pointsRef[0][2]
        x02 = self.pointsRef[1][0]
        y02 = self.pointsRef[1][1]
        z02 = self.pointsRef[1][2]
        x03 = self.pointsRef[2][0]
        y03 = self.pointsRef[2][1]
        z03 = self.pointsRef[2][2]

        x11 = self.points[0][0]
        y11 = self.points[0][1]
        z11 = self.points[0][2]
        x12 = self.points[1][0]
        y12 = self.points[1][1]
        z12 = self.points[1][2]
        x13 = self.points[2][0]
        y13 = self.points[2][1]
        z13 = self.points[2][2]

        rsq1 = x11 ** 2 + y11 ** 2 + z11 ** 2
        rsq2 = x12 ** 2 + y12 ** 2 + z12 ** 2
        rsq3 = x13 ** 2 + y13 ** 2 + z13 ** 2

        try:
            results = solve([(x - x01) ** 2 + (y - y01) ** 2 + (z - z01) ** 2 - rsq1,
                            (x - x02) ** 2 + (y - y02) ** 2 + (z - z02) ** 2 - rsq2,
                            (x - x03) ** 2 + (y - y03) ** 2 + (z - z03) ** 2 - rsq3], [x, y, z])

        except:
            print "calculating position failed"
            return

        for result in results:
            if result[1] > 0 and result[2] < 0:
                self.cameraPos = (result[0], result[1], result[2])
                print self.cameraPos
                break

        first = [[]]


        x21 = x01 - result[0]
        y21 = y01 - result[1]
        z21 = z01 - result[2]

        x22 = x02 - result[0]
        y22 = y02 - result[1]
        z22 = z02 - result[2]

        x23 = x03 - result[0]
        y23 = y03 - result[1]
        z23 = z03 - result[2]

        first = np.array([[x21, y21, z21], [x22, y22, z22], [x23, y23, z23]])
        second = np.array([[x11, y11, z11], [x12, y12, z12], [x13, y13, z13]])
        self.cameraQuaternion = self.get_quaternion(first, second)

        self.done = True

    def get_quaternion(self, lst1,lst2):

        matchlist=range(len(lst1))
        M=np.matrix([[0,0,0],[0,0,0],[0,0,0]])

        for i,coord1 in enumerate(lst1):
            x=np.matrix(np.outer(coord1,lst2[matchlist[i]]))
            M=M+x

        N11=float(M[0][:,0]+M[1][:,1]+M[2][:,2])
        N22=float(M[0][:,0]-M[1][:,1]-M[2][:,2])
        N33=float(-M[0][:,0]+M[1][:,1]-M[2][:,2])
        N44=float(-M[0][:,0]-M[1][:,1]+M[2][:,2])
        N12=float(M[1][:,2]-M[2][:,1])
        N13=float(M[2][:,0]-M[0][:,2])
        N14=float(M[0][:,1]-M[1][:,0])
        N21=float(N12)
        N23=float(M[0][:,1]+M[1][:,0])
        N24=float(M[2][:,0]+M[0][:,2])
        N31=float(N13)
        N32=float(N23)
        N34=float(M[1][:,2]+M[2][:,1])
        N41=float(N14)
        N42=float(N24)
        N43=float(N34)

        N=np.matrix([[N11,N12,N13,N14],
                      [N21,N22,N23,N24],
                      [N31,N32,N33,N34],
                      [N41,N42,N43,N44]])


        values,vectors=np.linalg.eig(N)
        w=list(values)
        mw=max(w)
        quat= vectors[:,w.index(mw)]
        quat=np.array(quat).reshape(-1,).tolist()
        return quat