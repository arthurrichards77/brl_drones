#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('brl_drones')
import brl_drones.rospid
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sqrt

rospy.init_node('testpid')
steering_pid = brl_drones.rospid.Rospid(0.15,0.0,0.22,'~steering')
vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist)

def pose_callback(data):
  # centre on 5,7
  dx = data.x - 5.0
  dy = data.y - 7.0
  # calculate radius
  radius = sqrt(dx*dx + dy*dy)
  # PID control for constant radius
  rospy.loginfo('Time is %f',rospy.get_rostime().to_sec())
  u = steering_pid.update(radius, 2.5, rospy.get_rostime().to_sec())
  # saturate
  u = brl_drones.rospid.saturate(u,0.3)
  # command
  v = Twist()
  v.linear.x = 0.3
  v.angular.z = -u
  # send it
  vel_pub.publish(v)
  # tell the world
  rospy.loginfo('Got pos = (%f,%f) rad=%f ctrl=%f', dx, dy, radius, u)

pose_sub = rospy.Subscriber('turtle1/pose', Pose, pose_callback)
rospy.spin()
