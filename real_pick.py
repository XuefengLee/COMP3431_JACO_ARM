#!/usr/bin/env python

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose,Quaternion
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'Arm'
GROUP_NAME_GRIPPER = 'Hand'

GRIPPER_FRAME = 'arm_stand'

GRIPPER_OPEN = [0]*4
GRIPPER_CLOSED = [0,0.5,0.5,0.5]
GRIPPER_NEUTRAL = [0] * 4

GRIPPER_JOINT_NAMES =  ['arm_5_joint','finger_joint_0', 'finger_joint_2', 'finger_joint_4']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'arm_stand'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        
                        
        # Initialize the move group for the right arm
        Arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        Arm.set_planner_id("RRTstarkConfigDefault")
        # Initialize the move group for the right gripper
        Hand = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Get the name of the end-effector link
        end_effector_link = Arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        Arm.set_goal_position_tolerance(0.05)
        Arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        Arm.allow_replanning(True)
        
        # Set the right arm reference frame
        Arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # Allow 5 seconds per planning attempt
        Arm.set_planning_time(10)
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 3
        
        # Set a limit on the number of place attempts
        max_place_attempts = 3
                
        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name        

        target_id = 'target'
 
                
        # Remove leftover objects from a previous run

        scene.remove_world_object(target_id)

        
        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, target_id)
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)
        

        
        # Open the gripper to the neutral position
        #Hand.set_joint_value_target(GRIPPER_NEUTRAL)
        #Hand.go()
       
        #rospy.sleep(1)


        target_size = [0.01, 0.01, 0.01]
        
        # Set the target pose in between the boxes and on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.03
        target_pose.pose.position.z = -0.3
        target_pose.pose.orientation.w = 1.0
        

        # Add the target object to the scene
        scene.add_box(target_id, target_pose, target_size)
        

        # Initialize the grasp pose to the target pose
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = REFERENCE_FRAME
        grasp_pose.pose = target_pose.pose
        grasp_pose.pose.position.y = 0.2
                
        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id])

        # Publish the grasp poses so they can be viewed in RViz
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)
    
        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = Arm.pick(target_id, grasps)
            rospy.sleep(0.2)
        
        
        

       
        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)
        
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
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.25, [0.0, -1.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.3, [0.0, 1.0, 0.0])
        
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
    


if __name__ == "__main__":
    MoveItDemo()