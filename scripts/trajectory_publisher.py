#!/usr/local/bin/env python
import rospy

from trajectory_msgs.msg import JointTrajectory

rospy.init_node('trajectory_publisher')

publishers = []

for sink in rospy.get_param('~sink_topics') :
	publishers.append( rospy.Publisher(sink, JointTrajectory, queue_size=10) )

def publishToTargets(data):
	global publishers
	for publisher in publishers :
		publisher.publish(data)

rospy.Subscriber( rospy.get_param('~source_topic'), JointTrajectory, publishToTargets)
rospy.spin()
