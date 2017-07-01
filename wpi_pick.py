#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import rospkg
import sys
import struct
import ctypes
import math
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped

import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import Bool,String
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from copy import deepcopy


GROUP_NAME_ARM = 'Arm'
GROUP_NAME_GRIPPER = 'Hand'

GRIPPER_FRAME = 'arm_stand'

GRIPPER_OPEN = [0]*3
GRIPPER_CLOSED = [0.65] * 3
GRIPPER_NEUTRAL = [0.1] * 3

GRIPPER_JOINT_NAMES =  ['jaco_finger_joint_0', 'jaco_finger_joint_2', 'jaco_finger_joint_4']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'arm_stand'



class Jaco_rapper():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.jaco_arm = MoveGroupCommander("Arm")
        self.hand = MoveGroupCommander("Hand")
        #self.pose_pub = rospy.Publisher("hand_pose", PoseStamped,queue_size = 100)

        self.pick_command = rospy.Publisher("pick_command", Bool, queue_size = 100)
        rospy.Subscriber("pick_pose",PoseStamped,self.pick)
        self.jaco_arm.allow_replanning(True)
        # Set the right arm reference frame
        self.jaco_arm.set_pose_reference_frame(REFERENCE_FRAME)
        self.jaco_arm.set_planning_time(5)
        self.jaco_arm.set_goal_tolerance(0.02)
        self.jaco_arm.set_goal_orientation_tolerance(0.1)

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

        target_id = 'target'
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5
        
        # Set a limit on the number of place attempts
        max_place_attempts = 5

        # Remove leftover objects from a previous run
        self.scene.remove_world_object(target_id)

        # Remove any attached objects from a previous session
        self.scene.remove_attached_object(GRIPPER_FRAME, target_id)
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)

        # Open the gripper to the neutral position
        self.hand.set_joint_value_target([0.2,0.2,0.2,0.2])
        self.hand.go()
       
        rospy.sleep(1)


        target_size = [0.01, 0.01, 0.01]
        
        # Set the target pose in between the boxes and on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = p.pose.position.x - 0.015
        #p.pose.position.x - 0.015
        target_pose.pose.position.y = 0.05
        target_pose.pose.position.z = p.pose.position.z
        target_pose.pose.orientation.w = 0
        
        print "Arm is catching {} object at ({}, {}, {})".format(p.header.frame_id, p.pose.position.x, p.pose.position.y, p.pose.position.z)
        # Add the target object to the scene
        self.scene.add_box(target_id, target_pose, target_size)
        

        # Initialize the grasp pose to the target pose
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = REFERENCE_FRAME
        grasp_pose.pose = target_pose.pose
        grasp_pose.pose.position.y = 0.24

        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id])

        # Publish the grasp poses so they can be viewed in RViz
        for grasp in grasps:
            #self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)
    
        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts

        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
                result = self.jaco_arm.pick(target_id, grasps)
                n_attempts += 1
                rospy.loginfo("Pick attempt: " +  str(n_attempts))
                rospy.sleep(1.0)
             
        # If the pick was successful, attempt the place operation   
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
              
            # Generate valid place poses
            

            places = self.make_places(p.header.frame_id)
              
            # Repeat until we succeed or run out of attempts
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                for place in places:
                    result = self.jaco_arm.place(target_id, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                rospy.sleep(0.2)
                  
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

        rospy.sleep(0.2)

        self.scene.remove_world_object(target_id)
       
        # Remove any attached objects from a previous session
        self.scene.remove_attached_object(GRIPPER_FRAME, target_id)

        self.pick_command.publish(Bool(True))
        return
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = 'arm_stand'
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.15, [0.0, -1.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, [0.0, 1.0, 0.0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]

        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        g.grasp_pose.pose.orientation = Quaternion(0.606301648371, 0.599731279995, 0.381153346104, 0.356991358063)
        # Set and id for this grasp (simply needs to be unique)
        g.id = str(len(yaw_vals))
        
        # Set the allowed touch objects to the input list
        g.allowed_touch_objects = allowed_touch_objects
        
        # Don't restrict contact force
        g.max_contact_force = 0

        grasps = [g]

        # Return the list
        return grasps

    # Generate a list of possible place poses
    def make_places(self,color):
        # Initialize the place location as a PoseStamped message
        x = 0
        z = 0

        if(color == 'red'):
            x = 0.4
            z = -0.4
        elif(color == 'blue'):
            x = 0.4
            z = -0.25
        else:
            x = -0.3
            z = -0.4

        place_pose = PoseStamped()
        place_pose.pose.position.x = x
        place_pose.pose.position.y = 0.2
        place_pose.pose.position.z = z
        # Start with the input place pose

        place_pose.header.frame_id = REFERENCE_FRAME
        # A list to hold the places
        places = []
        


        # Create a quaternion from the Euler angles
        
        place_pose.pose.orientation = Quaternion(0,0,0,0)

        # Append this place pose to the list
        places.append(deepcopy(place_pose))
        
        # Return the list
        return places


if __name__ == '__main__':

    rospy.init_node('jaco_arm')
    pose_pub = rospy.Publisher("hand_pose", PoseStamped, queue_size=10)
    jaco_rapper = Jaco_rapper()

    jaco_rapper.pick_command.publish(Bool(True))

    place_pose = PoseStamped()
    place_pose.pose.position.x = 0
    place_pose.pose.position.y = 0.2
    place_pose.pose.position.z = -0.4
    # Start with the input place pose

    place_pose.header.frame_id = 'red'

    jaco_rapper.pick(place_pose)
   # rospy.spin()
