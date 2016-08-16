#!/usr/bin/env python
import roslib
roslib.load_manifest('brl_drones')
import rospy
import csv

# set up the basics - number of drones and names

num_drones = rospy.get_param('num_drones')
print "Preparing files for %d drones" % num_drones

drone_names = ["drone%d" % (ii+1) for ii in range(num_drones)]
print "Drone names will be %s" % drone_names

output_package_folder = rospy.get_param('pkg_folder')
print "Package folder will be %s" % output_package_folder

# prepare the URDF files

urdf_file_name = rospy.get_param('urdf_file')
print "Preparing to write URDF file: %s" % urdf_file_name
f = open(urdf_file_name,'w')

preamble="""<?xml version="1.0"?>
  <robot name="drone" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find brl_drones)/urdf/drone_base.urdf.xacro" />

  <link name="base_link" />

"""

f.write(preamble)

for drone in drone_names:
  f.write("  <xacro:drone_base prefix=\"%s_\" base_link_name=\"base_link\" />\n\n" % drone)

f.write('</robot>\n')
f.close()
print "Finished writing URDF file."

# now for the launch file
launch_file_name = rospy.get_param('launch_file')
print "Preparing to write LAUNCH file: %s" % launch_file_name
f = open(launch_file_name,'w')

joint_state_list = [("%s/joint_states" % drone) for drone in drone_names]

preamble="""<launch>	

  <arg name="panels" default="false" />

  <param name="default_trajectory_folder"  value="$(find brl_drones)/launch" />
  <node name="traj_gui" pkg="brl_drones" type="csv_trajectory_gui.py" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find brl_drones)/urdf/drones.rviz"/>

  <param name="use_gui" value="false"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <rosparam param="source_list">%s</rosparam>
  </node>


""" % joint_state_list

f.write(preamble)

f.write("<param name=\"robot_description\" command=\"$(find xacro)/xacro.py %s\" />\n\n" % urdf_file_name)

for drone in drone_names:
  f.write("""  <include file="$(find brl_drones)/launch/single_virtual.launch">
    <arg name="drone_name" value="%s" />
    <arg name="use_panel" value="$(arg panels)" />
  </include>

  """ % drone)

f.write('</launch>\n')
f.close()
print "Finished writing LAUNCH file."

# finally an example trajectory CSV file
traj_file_name = rospy.get_param('traj_csv_file')
print "Preparing to write CSV file: %s" % traj_file_name
joint_list = [drone+joint for drone in drone_names for joint in ["_move_x","_move_y","_move_z","_turn_z"]]
print "Joint list is %s" % joint_list
with open(traj_file_name,'wb') as csvfile:
  csvwriter = csv.writer(csvfile)
  csvwriter.writerow(["time_from_start"]+joint_list)
  for ii in range(30):
    tt = ii+1
    thisrow = [tt]
    for dd in range(num_drones):
      thisrow += [dd*0.1*tt,dd*1.0,tt/30.0,dd*0.3]
    csvwriter.writerow(thisrow)

print "Finished writing CSV file."
print "SUCCESS: to execute >> roslaunch %s" % launch_file_name
