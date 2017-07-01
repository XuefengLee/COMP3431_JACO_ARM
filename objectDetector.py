#!/usr/bin/env python
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
from std_msgs.msg import Bool,String

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from copy import deepcopy
import Tkinter as tki
from PIL import ImageTk

from jaco_rapper import *
from object import *
from calibration import *


#MAX_OBJECTS = 10

GROUP_NAME_ARM = 'Arm'
GROUP_NAME_GRIPPER = 'Hand'
GRIPPER_FRAME = 'tabletop_ontop'

GRIPPER_OPEN = [0] * 4

GRIPPER_CLOSED = [0, 0.25, 0.25, 0.25]
GRIPPER_JOINT_NAMES = ['arm_5_joint', 'finger_joint_0', 'finger_joint_2', 'finger_joint_4']
GRIPPER_EFFORT = [1.0]
STATE_CALIBRATE = 0
STATE_OPERATE = 1

class objectDetect:
    objects = []
    containers = []

    disableImageUpdate = False
    armReady = True

    def __init__(self, feature="Color"):
        self.cubicPose2 = PoseStamped()
        #self.jaco_rapper = Jaco_rapper()
        self.calibration = calibration()
        self.node_name = "objectDetector"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.image_callback)
        self.cameraInfo_sub = rospy.Subscriber("/kinect2/sd/camera_info", CameraInfo, self.cameraInfo_callback)
        self.pcl_sub = rospy.Subscriber("/kinect2/sd/points", PointCloud2, self.pcl_callback)

        self.pick_sub = rospy.Subscriber("pick_command", Bool, self.pick_callback)

        self.pos_pub = rospy.Publisher('pick_pose', PoseStamped, queue_size=100)
        self.info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=100)



        self.transListen = tf.TransformListener()
        self.doPick = False

        self.root = tki.Tk()
        self.panel = None

        self.COLOR_RANGES = {"pink": {"low": np.array([145, 100, 100]), "high": np.array([165, 255, 255])},
                             "yellow": {"low": np.array([20, 100, 100]), "high": np.array([35, 255, 255])},
                             "green": {"low": np.array([45, 100, 50]), "high": np.array([75, 255, 255])},
                             "blue": {"low": np.array([100, 150, 0]), "high": np.array([140, 255, 255])},
                             "white": {"low": np.array([0, 0, 200]), "high": np.array([180, 255, 255])},
                             "red": {"low": np.array([160, 100, 100]), "high": np.array([179, 255, 255])}
                             }

        self.state = STATE_CALIBRATE

        self.tk_setup()

    def pick_callback(self, ready):
        self.armReady = ready

    def tk_setup(self):
        '''create the gui'''
        self.tk_create_widgets()
        self.tk_setup_layout()


    def tk_create_widgets(self):
        '''create the widgets used in the gui'''
        self.lbl = tki.Label(self.root)

        self.button_yes = tki.Button(self.root, text="Yes", command=self.startCalibrate)
        self.button_no = tki.Button(self.root, text="No", command=self.choose_other_points)
        self.textbox1_lblx = tki.Label(self.root, text="x:")
        self.textbox1_x = tki.Entry(self.root)
        self.textbox1_lbly = tki.Label(self.root, text="y:")

        self.textbox1_y = tki.Entry(self.root)
        self.textbox1_lblz = tki.Label(self.root, text="z:")

        self.textbox1_z = tki.Entry(self.root)
        self.textbox2_lblx = tki.Label(self.root, text="x:")

        self.textbox2_x = tki.Entry(self.root)
        self.textbox2_lbly = tki.Label(self.root, text="y:")

        self.textbox2_y = tki.Entry(self.root)
        self.textbox2_lblz = tki.Label(self.root, text="z:")

        self.textbox2_z = tki.Entry(self.root)
        self.textbox3_lblx = tki.Label(self.root, text="x:")

        self.textbox3_x = tki.Entry(self.root)
        self.textbox3_lbly = tki.Label(self.root, text="y:")

        self.textbox3_y = tki.Entry(self.root)
        self.textbox3_lblz = tki.Label(self.root, text="z:")

        self.textbox3_z = tki.Entry(self.root)

        self.lbl1 = tki.Label(self.root)
        self.lbl2 = tki.Label(self.root)
        self.lbl3 = tki.Label(self.root)



    def tk_setup_layout(self):
        '''set up the layout of the gui'''
        self.root.grid_rowconfigure(1, weight=0)
        self.root.grid_columnconfigure(0, weight=0)
        self.root.grid_columnconfigure(4, weight=0)
        #self.panel.grid(row=0, columnspan=5, rowspan=5)

        self.lbl1.grid(row = 5, columnspan=5)
        self.textbox1_lblx.grid(row=6, column=0)
        self.textbox1_x.grid(row=6, column=1)
        self.textbox1_lbly.grid(row=6, column=2)
        self.textbox1_y.grid(row=6, column=3)
        self.textbox1_lblz.grid(row=6, column=4)
        self.textbox1_z.grid(row=6, column=5)

        self.lbl2.grid(row = 7, columnspan=5)
        self.textbox2_lblx.grid(row=8, column=0)
        self.textbox2_x.grid(row=8, column=1)
        self.textbox2_lbly.grid(row=8, column=2)
        self.textbox2_y.grid(row=8, column=3)
        self.textbox2_lblz.grid(row=8, column=4)
        self.textbox2_z.grid(row=8, column=5)
        self.lbl3.grid(row = 9, columnspan=5)
        self.textbox3_lblx.grid(row=10, column=0)
        self.textbox3_x.grid(row=10, column=1)
        self.textbox3_lbly.grid(row=10, column=2)
        self.textbox3_y.grid(row=10, column=3)
        self.textbox3_lblz.grid(row=10, column=4)
        self.textbox3_z.grid(row=10, column=5)

        self.lbl.grid(row = 11, columnspan=5)

        self.button_yes.grid(row=12, column=1)
        self.button_no.grid(row=12, column=3)



    def startCalibrate(self):
        for object in self.objects:
            if object.inRange:
                if object.seq == 0:
                    object.x0 = float(self.textbox1_x.get())
                    object.y0 = float(self.textbox1_y.get())
                    object.z0 = float(self.textbox1_z.get())
                elif object.seq == 1:
                    object.x0 = float(self.textbox2_x.get())
                    object.y0 = float(self.textbox2_y.get())
                    object.z0 = float(self.textbox2_z.get())
                elif object.seq == 2:
                    object.x0 = float(self.textbox3_x.get())
                    object.y0 = float(self.textbox3_y.get())
                    object.z0 = float(self.textbox3_z.get())

                self.calibration.points.append((object.px, object.py, object.pz))
                self.calibration.pointsRef.append((object.x0, object.y0, object.z0))

        self.calibration.calPos()

        if self.calibration.done:
            self.disableImageUpdate = False
            self.state = STATE_OPERATE

            #hide the widgets for calibration
            self.textbox1_lblx.pack_forget()
            self.textbox1_lbly.pack_forget()
            self.textbox1_lblz.pack_forget()
            self.textbox1_x.pack_forget()
            self.textbox1_y.pack_forget()
            self.textbox1_z.pack_forget()

            self.textbox2_lblx.pack_forget()
            self.textbox2_lbly.pack_forget()
            self.textbox2_lblz.pack_forget()
            self.textbox2_x.pack_forget()
            self.textbox2_y.pack_forget()
            self.textbox2_z.pack_forget()

            self.textbox3_lblx.pack_forget()
            self.textbox3_lbly.pack_forget()
            self.textbox3_lblz.pack_forget()
            self.textbox3_x.pack_forget()
            self.textbox3_y.pack_forget()
            self.textbox3_z.pack_forget()

            self.lbl.configure(text="Operating mode")
            self.lbl1.pack_forget()
            self.lbl2.pack_forget()
            self.lbl3.pack_forget()

            self.button_yes.pack_forget()
            self.button_no.pack_forget()

            self.rbtnPick = tki.Radiobutton(self.root, text="Pick", padx=20, variable=self.doPick, value=True, command=self.enablePick)
            self.rbtnNotPick = tki.Radiobutton(self.root, text="Hold On", padx=20, variable=self.doPick, value=False, command=self.disablePick)
            self.rbtnPick.grid(row=13, columnspan=3)
            self.rbtnNotPick.grid(row=14, columnspan=3)

    def enablePick(self):
        self.doPick = True

    def disablePick(self):
        self.doPick = False

    def choose_other_points(self):
        '''Re-choose points to calibrate'''
        self.disableImageUpdate = False
        self.objects = []


    def image_callback(self, image):
        #print image.header.stamp
        if image.header.stamp - rospy.Time.now() > rospy.Duration(1.0):
            return
        try:
            # image = self.uncompressImage(compressedImage, "bgr8")
            image_bgr = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        rects = []
        colors = []
        #boxes = []

        if not self.disableImageUpdate:
            #find the rects as candidates to be picked up
            for obj in self.objects:
                obj.exist = False
            #Find the objects with colors
            for k in self.COLOR_RANGES:

                if k != "white":
                    rects = self.findColor(image, k, 20)

                    i = 0
                    for rect in rects:
                        if rect[2] * rect[3] > 100:
                            object = Object()
                            object.x = rect[0]
                            object.y = rect[1]
                            object.w = rect[2]
                            object.h = rect[3]
                            object.color = k
                            obj = self.exist(object)
                            if  obj == None:
                                object.exist = True
                                self.objects.append(object)
                            else:
                                obj.exist = True
                            i += 1

            # Remove object from the lists if it is no longer seen
            if self.state == STATE_OPERATE:
                self.clearObjects()

            #Show detected objects who is within defined range
            for object in self.objects:
                if object.inRange:
                    cv2.rectangle(image_bgr, (object.x, object.y), (object.x + object.w, object.y + object.h), (255, 255, 255), 3)
                    cv2.putText(image_bgr, object.color, (object.x, object.y - 20), cv.CV_FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))


        #Mark the detected object with white rectangles
        else:
            for object in self.objects:
                if object.inRange:
                    cv2.rectangle(image_bgr, (object.x, object.y), (object.x + object.w, object.y + object.h), (255, 255, 255), 3)
                    cv2.putText(image_bgr, object.color, (object.x, object.y - 20), cv.CV_FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))

        cv2.imshow("rgb", image_bgr)
        cv.WaitKey(5)


    def exist(self, obj):
        #Judge if the object has already been detected and stored
        for object in self.objects:
            if (object.x - obj.x)**2 + (object.y - obj.y)**2 < 100:
                return object
        return None

    def clearObjects(self):
        self.objects = [x for x in self.objects if x.exist]

    def cameraInfo_callback(self, cameraInfo):

        self.info_pub.publish(cameraInfo)

    def pcl_callback(self, point_cloud):
        if point_cloud.header.stamp - rospy.Time.now() > rospy.Duration(1.0):
            return

        if self.disableImageUpdate:
            return

        gen = pc2.read_points(point_cloud, skip_nans=False)
        width = point_cloud.width
        height = point_cloud.height

        int_data = list(gen)
        pclimage = np.zeros((height, width, 3), dtype=np.float32)

        i = 0
        for x in int_data:
            # print r, g, b
            if math.isnan(x[0]) or math.isnan(x[1]) or math.isnan(x[2]):
                x = (0, 0, 0)
            pclimage[i / width][i % width] = (x[0], x[1], x[2])
            i = i + 1

        for object in self.objects:
            position = pclimage[object.y + object.h / 2][object.x + object.w / 2]

            if position[2] < 1.5 and position[2] <> 0:
                object.inRange = True
                print "Start"
                print "({}, {}): ({})".format(object.y + object.h / 2, object.x + object.w / 2,
                                              pclimage[object.y + object.h / 2][object.x + object.w / 2])
                print "End"
                object.px = position[0]
                object.py = position[1]
                object.pz = position[2]

            else:
                object.inRange = False

        if self.state == STATE_CALIBRATE:
            self.calibrate()

        elif self.state == STATE_OPERATE:
            self.transform()
        else:
            print "({}), ({})".format(self.calibration.cameraPos, self.calibration.cameraQuaternion)

    def calibrate(self):
        i = 0
        self.lbl.configure(text="Use these three points?")

        for object in self.objects:
            if object.inRange:
                self.find_own_pos(object.px, object.py, object.pz, object.color, i)
                object.seq = i
                i += 1
            if i == 3 and self.state == STATE_CALIBRATE:
                break

        if i == 3:
            self.disableImageUpdate = True

    def transform(self):
        now = rospy.Time.now()
        br = tf.TransformBroadcaster()
        br.sendTransform((self.calibration.cameraPos[0], self.calibration.cameraPos[1], self.calibration.cameraPos[2]),
                         (self.calibration.cameraQuaternion[1], self.calibration.cameraQuaternion[2], self.calibration.cameraQuaternion[3], self.calibration.cameraQuaternion[0]),
                         now,
                         "calibration",
                         "arm_stand")
        for object in self.objects:
            if object.inRange:
                self.transformPoint(object, now)
                break

    def transformPoint(self, object, timeStamp):
        x = object.px #the (x, y, z) in pointcloud
        y = object.py
        z = object.pz

        cubicPose = PoseStamped()
        now = timeStamp

        cubicPose.header.stamp = now
        cubicPose.header.frame_id = 'calibration'
        cubicPose.pose.position.x = x
        cubicPose.pose.position.y = y
        cubicPose.pose.position.z = z
        cubicPose.pose.orientation.x = 0.0
        cubicPose.pose.orientation.y = 0.0
        cubicPose.pose.orientation.z = 0.0
        cubicPose.pose.orientation.w = 1.0

        try:
            self.transListen.waitForTransform('calibration', 'arm_stand', now, rospy.Duration(4.0))
        except:
            return
        cubicPose2 = self.transListen.transformPose('arm_stand', cubicPose)
        cubicPose2.header.frame_id = object.color
        print(cubicPose2)
        if self.doPick:
            #self.jaco_rapper.pick(cubicPose2.pose.position.x, cubicPose2.pose.position.y, cubicPose2.pose.position.z, object.color,rospy)
            if self.armReady:
                self.pos_pub.publish(cubicPose2)
                self.armReady = False
        else:
            print "Thinking of picking up {} block at ({}, {}, {}).".format(object.color, cubicPose2.pose.position.x, cubicPose2.pose.position.y, cubicPose2.pose.position.z)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

    def findColor(self, image, color, minSize):
        rect = [0] * 4
        rects = []
        low = self.COLOR_RANGES[color]["low"]
        high = self.COLOR_RANGES[color]["high"]
        mask = cv2.inRange(image, low, high)
        output = cv2.bitwise_and(image, image, mask=mask)
        output1 = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        output2 = cv2.cvtColor(output1, cv2.COLOR_BGR2GRAY)
        # print(output)
        ret, thresh = cv2.threshold(output2, 10, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)
        i = 0
        for contour in contours:
            cnt = contour
            x0, y0, w0, h0 = cv2.boundingRect(cnt)
            if w0 * h0 < minSize:
                continue
            rect = [x0, y0, w0, h0]
            rects.append(rect)
            i = i + 1
        num = i
        return rects

    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        return edges

    # Calibrate the camera's position
    def find_own_pos(self, x, y, z, color, seq):
        if color == "red":
            x0 = 0
            y0 = 0.03
            z0 = -0.72
        elif color == "blue":
            x0 = 0.135
            y0 = 0.035
            z0 = -0.05
        elif color == "green":
            x0 = -0.135
            y0 = 0.025
            z0 = -0.05

        if seq == 0:
            self.lbl1.configure(text = "Detected calibration position ({}) \n ({}, {}, {}).".format(color, x, y, z))
            self.textbox1_x.delete(0, tki.END)
            self.textbox1_x.insert(0, "{}".format(x0))
            self.textbox1_y.delete(0, tki.END)
            self.textbox1_y.insert(0, "{}".format(y0))
            self.textbox1_z.delete(0, tki.END)
            self.textbox1_z.insert(0, "{}".format(z0))
        elif seq == 1:
            self.lbl2.configure(text = "Detected calibration position ({}) \n ({}, {}, {}).".format(color, x, y, z))
            self.textbox2_x.delete(0, tki.END)
            self.textbox2_x.insert(0, "{}".format(x0))
            self.textbox2_y.delete(0, tki.END)
            self.textbox2_y.insert(0, "{}".format(y0))
            self.textbox2_z.delete(0, tki.END)
            self.textbox2_z.insert(0, "{}".format(z0))
        elif seq == 2:
            self.lbl3.configure(text = "Detected calibration position ({}) \n ({}, {}, {}).".format(color, x, y, z))
            self.textbox3_x.delete(0, tki.END)
            self.textbox3_x.insert(0, "{}".format(x0))
            self.textbox3_y.delete(0, tki.END)
            self.textbox3_y.insert(0, "{}".format(y0))
            self.textbox3_z.delete(0, tki.END)
            self.textbox3_z.insert(0, "{}".format(z0))

        return

def main(args):
    try:
        print(tf.transformations.euler_from_quaternion((-0.9991509151752196, 0.006177326394527312, -0.03775980328552767, 0.01528026828871064)))
        obj = objectDetect()
        obj.root.mainloop()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()
    #since we have mainloop(), rospy.spin() is not required

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

if __name__ == '__main__':
    main(sys.argv)