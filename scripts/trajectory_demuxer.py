#!/usr/local/bin/env python
#
# trajectory_demuxer.py
# Splits a JointTrajectory message containing information for multiple drones
#  into a set of JointTrajectory messages for each drone
# Takes a parameter in YAML form for the destination topics:
#  <rosparam param="destination_topics">{drone1: 'drone1/cmd_traj', drone2: 'drone2/cmd_traj', drone3: 'drone3/cmd_traj'}</rosparam>
# And a standard ros parameter for the source topic:
#  <param name="source_topic" value="bundled_traj" />
# Expects that the source topic will publish a JointTrajectory message containing
#  all combinations of [droneName]_[jointName]
# By default joint names are: ["move_x","move_y","move_z","turn_z"]
#  The parameter ~joint_names can be set to override this:
#  <rosparam param="joint_names">[ 'xPos', 'yPos', 'zPos', 'heading']</rosparam>
# This will override both the input and output joint names, to override the
#  output names, the parameter ~output_joint_names can be set in the same fashion

import rospy

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('trajectory_muxer')

namePublisherPairs = [];

def extractDataForName(name,data):
	global outJointNames
	global inJointNames
	positionIdxs = []
	for jointName in inJointNames :
		positionIdxs.append( data.joint_names.index(name+"_"+jointName) )

	outData = JointTrajectory()
	outData.joint_names = outJointNames

	for point in data.points :
		outPoint = JointTrajectoryPoint()

		for index in positionIdxs :
			outPoint.positions.append(point.positions[index])

		outData.points.append(outPoint)

	return outData

def publishToTargets(data):
	global namePublisherPairs
	for npPair in namePublisherPairs :
		outData = extractDataForName(npPair.name,data)
		npPair.publisher.publish(outData)

# Check for presence of required parameters
if !rospy.has_param('~destination_topics'):
	rospy.logfatal('destination_topics not specified')

if !rospy.has_param('~source_topic'):
	rospy.logfatal('source_topic not specified')

if !rospy.has_param('~joint_names'):
	inJointNames = ["move_x","move_y","move_z","turn_z"]
else:
	jointNames = rospy.get_param('~joint_names')

if !rospy.has_param('~output_joint_names'):
	outJointNames = inJointNames
else:
	outJointNames = rospy.get_param('~output_joint_names')

# Get a list of drone names and topics to sink to
for name,topic in rospy.get_param('~destination_topics'):
	# For each pair, append a {name:,publisher:} dictionary to namePublisherPairs
	namePublisherPairs.append( {
		name      : name,
		publisher : rospy.Publisher(topic, JointTrajectory, queue_size=10)
		} )

rospy.Subscriber( rospy.get_param('~source_topic'), JointTrajectory, publishToTargets)
rospy.spin()
