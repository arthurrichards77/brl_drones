#!/usr/bin/env python
import roslib
roslib.load_manifest('brl_drones')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import csv
import tkFileDialog
from Tkinter import *

class TrajGui:

  def __init__(self, master):
    frame=Frame(master, bg="yellow")
    master.title("Trajectory")
    frame.pack()
    self.my_frame = frame
    self.traj = None
    self.pub_traj = rospy.Publisher('cmd_traj', JointTrajectory)
    self.repeat_traj = False

    # initial directory to look for files
    self.initdir = rospy.get_param('default_trajectory_folder','~')

    # repeat period - negative means never
    self.repeat_period = rospy.get_param('repeat_period',-1.0)

    # load button
    self.quit_button = Button(frame, text="Load", command=self.load_button)
    self.quit_button.grid(row=0, column=0, padx=10, pady=10)

    # run button
    self.quit_button = Button(frame, text="Run", command=self.run_button)
    self.quit_button.grid(row=0, column=1, padx=10, pady=10)

    # quit button
    self.quit_button = Button(frame, text="Quit", command=frame.quit)
    self.quit_button.grid(row=0, column=2, padx=10, pady=10)

  def load_csv_traj(self,file_name):
    # prepare empty trajectory
    self.traj = JointTrajectory()
    # open the file
    with open(file_name, 'rb') as csv_file:
      traj_reader = csv.reader(csv_file)
      # get first row for names
      first_row = traj_reader.next()
      assert(first_row[0]=='time_from_start')
      self.traj.joint_names=first_row[1:]
      # then subsequent rows
      for row in traj_reader:
        assert(len(row)==len(first_row))
        new_point = JointTrajectoryPoint()
        new_point.time_from_start=rospy.Duration(float(row[0]))
        new_point.positions = [float(p) for p in row[1:]]
        self.traj.points += [new_point]
    rospy.loginfo("Trajectory loaded from CSV file")    

  def check_ros(self):
    if rospy.is_shutdown():
      self.my_frame.quit()
    else:
      root.after(1000,self.check_ros)

  def load_button(self):
    # always cancels repeat
    self.repeat_traj = False
    # default file name
    file_name=tkFileDialog.askopenfilename(initialdir=self.initdir)
    if file_name:
      self.load_csv_traj(file_name)

  def run_button(self):
    if self.traj:
      rospy.loginfo("Publishing trajectory")
      self.pub_traj.publish(self.traj)
      # and turn on repeating
      if self.repeat_period > 0.0:
        self.repeat_traj = True
        root.after(1000*self.repeat_period,app.check_repeat)

  def check_repeat(self):
    if self.repeat_traj:
      self.run_button()

rospy.init_node('trajectory_gui', anonymous=True)
root = Tk()
app = TrajGui(root)
root.after(1000,app.check_ros)
root.mainloop()
root.destroy()
