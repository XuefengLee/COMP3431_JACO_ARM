#!/usr/bin/env python
import sys, rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node("spawn_chessboard")
  rospy.wait_for_service("gazebo/delete_model")
  rospy.wait_for_service("gazebo/spawn_sdf_model")
  
delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel) 
delete_model("chessboard")
s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0)) 
board_pose = Pose(Point(0, 0 ,1), orient)
unit = 0.05
with open("chess_board.sdf", "r") as f:
  board_xml = f.read()
with open("chess_piece.sdf", "r") as f:
  piece_xml = f.read()
print s("chessboard", board_xml, "", board_pose, "world")
for row in [0,1,6,7]:
  for col in xrange(0,8):
    piece_name = "piece_%d_%d" % (row, col)
    delete_model(piece_name)
    pose = deepcopy(board_pose)
    pose.position.x = board_pose.position.x - 3.5 * unit + col * unit
    pose.position.y = board_pose.position.y - 3.5 * unit + row * unit
    pose.position.z += 0.02
    s(piece_name, piece_xml, "", pose, "world")
