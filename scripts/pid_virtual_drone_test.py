#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('brl_drones')
import brl_drones.rospid

rospy.init_node('testpid')
# control drone2 to follow drone 1
quad_pid = brl_drones.rospid.Quadpid('drone2_hull','drone1_hull',ctrl_topic='/drone2/cmd_vel')

rate = rospy.Rate(10)
while not rospy.is_shutdown():
  quad_pid.update(rospy.get_rostime().to_sec())
  rate.sleep()

