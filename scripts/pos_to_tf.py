#!/usr/bin/env python
import roslib
roslib.load_manifest('brl_drones')
import rospy
import tf
import numpy
from math import sqrt
from geometry_msgs.msg import Vector3

class PointerCtrl:

  def __init__(self):
    rospy.init_node('pos_to_tf', anonymous=True)
    self.listener = tf.TransformListener()
    self.br = tf.TransformBroadcaster()
    self.targ_trans = (0,0,1)
    self.targ_rot = (0,0,0,1)
    # frame names
    self.static_frame = rospy.get_param('static_frame','world')
    self.target_frame = rospy.get_param('target_frame','target')

  def run(self):
    self.sub = rospy.Subscriber('input_pos', Vector3, self.pos_callback)

  def pos_callback(self,data):
      self.targ_trans = (data.x,data.y,data.z)
      self.br.sendTransform(self.targ_trans,
                            self.targ_rot,
                            rospy.Time.now(),
                            self.target_frame,
                            self.static_frame)
  
pc = PointerCtrl()
pc.run()
rospy.spin()

