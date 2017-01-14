#!/usr/bin/env python
import roslib
roslib.load_manifest('brl_drones')
import rospy
import tf
import numpy

# update rate and time step
rospy.init_node('pointer_ctrl', anonymous=True)
rate = rospy.Rate(10)

listener = tf.TransformListener()
br = tf.TransformBroadcaster()

targ_trans = (0,0,1)
targ_rot = (0,0,0,1)

while not rospy.is_shutdown():
  try:
    (trans,rot) = listener.lookupTransform('base_link', 'xyzrqp', rospy.Time(0))
    M44 = tf.transformations.quaternion_matrix(rot)
    M33 = M44[:3,:3]
    targ_trans = numpy.add(trans,numpy.dot(M33,(3,0,0)))
    print targ_trans
    #euls = tf.transformations.euler_from_quaternion(rot)
    #print euls
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print 'problem'
  br.sendTransform(targ_trans,
                     targ_rot,
                     rospy.Time.now(),
                     "target",
                     "base_link")
  rate.sleep()
  
