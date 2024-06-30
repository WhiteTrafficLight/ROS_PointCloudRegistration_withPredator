#!/usr/bin/env python3.8
import rospy
from moveit_commander import PlanningSceneInterface
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header

def octomap_callback(msg):
    planning_scene_interface = PlanningSceneInterface()
    header = Header(frame_id="base_link")
    planning_scene_interface.apply_octomap(msg, header)

if __name__ == "__main__":
    rospy.init_node('update_planning_scene')
    rospy.Subscriber("/octomap_full", Octomap, octomap_callback)
    rospy.spin()



