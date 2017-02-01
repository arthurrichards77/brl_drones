#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('brl_drones')
import brl_drones.rospid

ctrl_drone_frame = rospy.get_param('ctrl_drone_frame','drone2_hull')
targ_drone_frame = rospy.get_param('targ_drone_frame','drone1_hull')

rospy.loginfo("Controlling %s to target %s", ctrl_drone_frame, targ_drone_frame)

rospy.init_node('drone_control')
# control drone2 to follow drone 1
quad_pid = brl_drones.rospid.Quadpid(ctrl_drone_frame, targ_drone_frame, ctrl_topic='ctrl_vel')

update_freq = rospy.get_param('update_frequency',10.0)
    
rate = rospy.Rate(update_freq)

while not rospy.is_shutdown():
  quad_pid.update(rospy.get_rostime().to_sec())
  rate.sleep()

