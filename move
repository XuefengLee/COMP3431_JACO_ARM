#!/usr/bin/env python
import sys, rospy, tf, moveit_commander, random
import moveit_msgs.msg
import geometry_msgs.msg
class Wrapper:
    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander("Arm")
        
    def setPose(self,deltaX,deltaY,deltaZ):
        current = self.group.get_current_pose().pose
        print(current)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = current.orientation
        pose_target.position.x = current.position.x + deltaX
        pose_target.position.y = current.position.y + deltaY
        pose_target.position.z = current.position.z + deltaZ
        print("set pose-----------------")
        print(pose_target)
        self.group.set_pose_target(pose_target)
        self.group.go(True)
        print(self.group.get_current_pose().pose)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv) 
    rospy.init_node('jaco_cli',anonymous=True)
    argv = rospy.myargv(argv=sys.argv) # filter out any arguments used by ROS 
    if len(argv) != 4:
        print "usage: r2_cli.py  Xdelta Ydelta Zdelta"
        sys.exit(1)
    r2w = Wrapper()
    r2w.setPose(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))
    
    print("after moving pose----------")
    #print(r2w.group.group.get_current_pose().pose)
    #print "============ Reference frame: %s" % r2w.group.get_planning_frame()
    #print "end effector: %s" % r2w.group.get_end_effector_link()

    moveit_commander.roscpp_shutdown()
