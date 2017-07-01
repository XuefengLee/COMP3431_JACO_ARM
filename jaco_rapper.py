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

class Jaco_rapper():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.jaco_arm = MoveGroupCommander("Arm")
        self.hand = MoveGroupCommander("Hand")
        self.pose_pub = rospy.Publisher("hand_pose", PoseStamped,queue_size = 100)

        self.pick_command = rospy.Publisher("pick_command", Bool, queue_size = 100)
        rospy.Subscriber("pick_pose",PoseStamped,self.pick)
        self.jaco_arm.allow_replanning(True)
        self.jaco_arm.set_planning_time(5)
        self.jaco_arm.set_goal_tolerance(0.02)
        self.jaco_arm.set_goal_orientation_tolerance(0.1)

        self.place_pose = PoseStamped()
        self.place_pose.header.frame_id = 'arm_stand'
        self.place_pose.pose.position.x = 0.4
        self.place_pose.pose.position.y = 0.4
        self.place_pose.pose.position.z = -0.4
        self.place_pose.pose.orientation = Quaternion(0.606301648371, 0.599731279995, 0.381153346104, 0.356991358063)
        self.orient = [2.042967990797618, -0.03399658412747265, 1.5807131823846676]
        self.result = False
        #self.pick_command.publish(True)

    def test(self):
        #self.hand.set_joint_value_target([0, 0, 0, 0])

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'arm_stand'
        grasp_pose.pose.position.x = 0
        grasp_pose.pose.position.y = 0.24
        grasp_pose.pose.position.z = -0.4
        grasp_pose.pose.orientation = Quaternion(0.606301648371, 0.599731279995, 0.381153346104, 0.356991358063)
       # self.hand.set_joint_value_target([0, 0.012 ,0.012 ,0.012])
        while(True):
            self.jaco_arm.set_pose_target(grasp_pose)  # move to the top of the target
            self.jaco_arm.go()
            rospy.sleep(0.2)
            #result = self.jaco_arm.go()




    def pick(self,p):
        #self.pick_command.publish(Bool(False))

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'arm_stand'
        grasp_pose.pose.position.x = p.pose.position.x - 0.015
        grasp_pose.pose.position.y = 0.5
        grasp_pose.pose.position.z = p.pose.position.z

        print "Arm is catching {} object at ({}, {}, {})".format(p.header.frame_id, p.pose.position.x, p.pose.position.y, p.pose.position.z)

        #self.orient[0] += 0.5
        # self.place_pose.pose.orientation =\
        #o = tf.transformations.quaternion_from_euler(self.orient[0], self.orient[1], self.orient[2])

        grasp_pose.pose.orientation= Quaternion(0.606301648371, 0.599731279995, 0.381153346104, 0.356991358063)

        #print(tf.transformations.euler_from_quaternion((0.606301648371, 0.599731279995, 0.381153346104, 0.356991358063)))
        #grasp_pose.pose.orientation = Quaternion(o[0], o[1], o[2], o[3])

        self.hand.set_joint_value_target([0, 0, 0, 0]) #open the hand
        self.hand.go()
        rospy.sleep(0.2)

        self.jaco_arm.set_pose_target(grasp_pose) #move to the top of the target

        result = self.jaco_arm.go()


        rospy.sleep(0.2)

        grasp_pose.pose.position.y = 0.24       #lower the arm to grasp
        self.pose_pub.publish(grasp_pose)
        self.jaco_arm.set_pose_target(grasp_pose)
        print (result)
        result = result and self.jaco_arm.go()
        print (result)

        if(result != True) :
            self.pick_command.publish(Bool(True))

            return

        rospy.sleep(0.2)
        self.hand.set_joint_value_target([0, 0.012 ,0.012 ,0.012])
        self.hand.go()
        rospy.sleep(0.2)

        grasp_pose.pose.position.y = 0.5
        self.jaco_arm.set_pose_target(grasp_pose)
        self.jaco_arm.go()
        rospy.sleep(0.2)

        self.place()

        self.pick_command.publish(Bool(True))

    def place(self):

        self.jaco_arm.set_pose_target(self.place_pose)
        self.jaco_arm.go()
        rospy.sleep(0.2)

        self.place_pose.pose.position.y = 0.3
        self.jaco_arm.set_pose_target(self.place_pose)
        self.jaco_arm.go()
        rospy.sleep(0.2)

        self.hand.set_joint_value_target([0, 0, 0, 0])


        self.place_pose.pose.position.y = 0.5
        self.jaco_arm.set_pose_target(self.place_pose)
        self.jaco_arm.go()
        rospy.sleep(0.2)

if __name__ == '__main__':

    rospy.init_node('jaco_arm')
    pose_pub = rospy.Publisher("hand_pose", PoseStamped, queue_size=10)
    jaco_rapper = Jaco_rapper()
    #jaco_rapper.test()
    #print("x:")
    #x = float(raw_input())
    #print("y:")
    #y = float(raw_input())
    #print("z:")
    #z = float(raw_input())
    #color = None
    #jaco_rapper.pick(x,y,z,color,rospy,pose_pub)
    jaco_rapper.pick_command.publish(Bool(True))
    rospy.spin()



'''
To do:
Choose objects to pick, ignore the ones out of the boundary of the table (using ARM_STAND frame)


'''


