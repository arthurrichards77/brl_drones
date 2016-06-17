#!/usr/bin/python
from Tkinter import *

import roslib
roslib.load_manifest('rospid')
import rospy
from std_msgs.msg import Float32

class App:

  def __init__(self, master):
    frame=Frame(master)
    frame.pack()

    # quit button
    self.quit_button = Button(frame, text="Quit", command=frame.quit)
    self.quit_button.grid(row=0, column=4)

    # gain entries and update buttons

    # start with kp
    self.kp_label = Label(frame, text='kp')
    self.kp_label.grid(row=1, column=2, padx=10, pady=10)    
    # text entry box
    self.kp_entry = Entry(frame)
    # see if parameter for initial setting exists, else use zero
    init_kp = rospy.get_param('init_gains/kp',0.0)
    self.kp_entry.insert(0,str(init_kp))
    self.kp_entry.grid(row=1, column=3, padx=10, pady=10)
    # run the update every time user presses return
    self.kp_entry.bind("<Return>", self.update_kp)    
    
    # now same for ki
    self.ki_label = Label(frame, text='ki')
    self.ki_label.grid(row=2, column=2, padx=10, pady=10)    
    self.ki_entry = Entry(frame)
    # see if parameter for initial setting exists, else use zero
    init_ki = rospy.get_param('init_gains/ki',0.0)
    self.ki_entry.insert(0,str(init_ki))
    self.ki_entry.grid(row=2, column=3, padx=10, pady=10)    
    self.ki_entry.bind("<Return>", self.update_ki)    
    
    # and of course kd
    self.kd_label = Label(frame, text='kd')
    self.kd_label.grid(row=3, column=2, padx=10, pady=10)    
    self.kd_entry = Entry(frame)
    # see if parameter for initial setting exists, else use zero
    init_kd = rospy.get_param('init_gains/kd',0.0)
    self.kd_entry.insert(0,str(init_kd))
    self.kd_entry.grid(row=3, column=3, padx=10, pady=10)    
    self.kd_entry.bind("<Return>", self.update_kd)    
    
    # gain publishers
    self.kp_pub = rospy.Publisher('tune_gains/kp',Float32)
    self.ki_pub = rospy.Publisher('tune_gains/ki',Float32)
    self.kd_pub = rospy.Publisher('tune_gains/kd',Float32)

  # callbacks for update events
  # note the event argument is optional
  # so these can be called by buttons or but event bindings
  def update_kp(self, event=''):
    self.kp_pub.publish(float(self.kp_entry.get()))

  def update_ki(self, event=''):
    self.ki_pub.publish(float(self.ki_entry.get()))

  def update_kd(self, event=''):
    self.kd_pub.publish(float(self.kd_entry.get()))

# "main" code - sloppy but ok for now
rospy.init_node('tune_gui', anonymous=True)
root = Tk()
# show the namespace in the window title
root.wm_title(rospy.get_namespace())
app = App(root)
root.mainloop()
root.destroy()
