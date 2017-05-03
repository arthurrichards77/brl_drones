#!/usr/bin/env python
import roslib
roslib.load_manifest('brl_drones')
import rospy
import tf
import numpy
from math import sqrt
import os

class PointerCtrl:

  def __init__(self):
    rospy.init_node('pointer_ctrl', anonymous=True)
    self.rate = rospy.Rate(10)
    self.listener = tf.TransformListener()
    self.br = tf.TransformBroadcaster()
    self.connected = False
    self.targ_trans = (0,0,1)
    self.targ_rot = (0,0,0,1)
    self.pointer_distance = 3.0
    # frame names
    self.static_frame = rospy.get_param('static_frame','world')
    self.pointer_frame = rospy.get_param('pointer_frame','pointer')
    self.target_frame = rospy.get_param('target_frame','target')
    self.drone_frame = rospy.get_param('drone_frame','drone')

  def run(self):
    while not rospy.is_shutdown():
      try:
        (trans,rot) = self.listener.lookupTransform(self.pointer_frame, self.drone_frame, rospy.Time(0))
        offset_tan = sqrt(trans[2]*trans[2] + trans[1]*trans[1])/trans[0]
        print offset_tan     
        if trans[0]>0 and offset_tan < 0.05:
          if self.connected==False:
            self.pointer_distance = trans[0]
            self.connected = True
            rospy.loginfo('CONNECTED!!')
            os.system('paplay /usr/share/sounds/ubuntu/stereo/phone-incoming-call.ogg')
        else:
          pass
        if self.connected == True:
          (trans,rot) = self.listener.lookupTransform(self.static_frame, self.pointer_frame, rospy.Time(0))
          M44 = tf.transformations.quaternion_matrix(rot)
          M33 = M44[:3,:3]
          if M33[2,1]*M33[2,1]>0.4:
            self.connected = False
            rospy.loginfo('DISCONNECTED!!')
            os.system('paplay /usr/share/sounds/freedesktop/stereo/bell.oga')
          else:
            self.targ_trans = numpy.add(trans,numpy.dot(M33,(self.pointer_distance,0,0)))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "Cannot find transform from %s to %s" % (self.pointer_frame,self.drone_frame)
      self.br.sendTransform(self.targ_trans,
                            self.targ_rot,
                            rospy.Time.now(),
                            self.target_frame,
                            self.static_frame)
      self.rate.sleep()
  
pc = PointerCtrl()
pc.run()

