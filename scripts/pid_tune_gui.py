#!/usr/bin/python
from Tkinter import *

import roslib
roslib.load_manifest('brl_drones')
import rospy
from std_msgs.msg import Float32

class App:

  def __init__(self, master):
    frame=Frame(master)
    frame.pack()
    self.my_frame = frame
    
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
    
    # upper limit
    self.up_label = Label(frame, text='upper')
    self.up_label.grid(row=4, column=2, padx=10, pady=10)    
    self.up_entry = Entry(frame)
    # see if parameter for initial setting exists, else use zero
    init_up = rospy.get_param('init_limits/upper',1.0)
    self.up_entry.insert(0,str(init_up))
    self.up_entry.grid(row=4, column=3, padx=10, pady=10)    
    self.up_entry.bind("<Return>", self.update_up)    
    
    # lower limit
    self.lo_label = Label(frame, text='lower')
    self.lo_label.grid(row=5, column=2, padx=10, pady=10)    
    self.lo_entry = Entry(frame)
    # see if parameter for initial setting exists, else use zero
    init_lo= rospy.get_param('init_limits/lower',-1.0)
    self.lo_entry.insert(0,str(init_lo))
    self.lo_entry.grid(row=5, column=3, padx=10, pady=10)    
    self.lo_entry.bind("<Return>", self.update_lo)    
    
    # lower limit
    self.rs_label = Label(frame, text='reset int')
    self.rs_label.grid(row=6, column=2, padx=10, pady=10)    
    self.rs_entry = Entry(frame)
    # see if parameter for initial setting exists, else use zero
    self.rs_entry.insert(0,str(0.0))
    self.rs_entry.grid(row=6, column=3, padx=10, pady=10)    
    self.rs_entry.bind("<Return>", self.update_rs)    
    
    # gain publishers
    self.kp_pub = rospy.Publisher('tune_gains/kp',Float32,queue_size=1)
    self.ki_pub = rospy.Publisher('tune_gains/ki',Float32,queue_size=1)
    self.kd_pub = rospy.Publisher('tune_gains/kd',Float32,queue_size=1)

    # limit and int publishers
    self.up_pub = rospy.Publisher('set_limits/upper',Float32,queue_size=1)
    self.lo_pub = rospy.Publisher('set_limits/lower',Float32,queue_size=1)
    self.rs_pub = rospy.Publisher('reset_integrator',Float32,queue_size=1)

  def check_ros(self):
    if rospy.is_shutdown():
      self.my_frame.quit()
    else:
      root.after(1000,app.check_ros)

  # callbacks for update events
  # note the event argument is optional
  # so these can be called by buttons or but event bindings
  def update_kp(self, event=''):
    self.kp_pub.publish(float(self.kp_entry.get()))

  def update_ki(self, event=''):
    self.ki_pub.publish(float(self.ki_entry.get()))

  def update_kd(self, event=''):
    self.kd_pub.publish(float(self.kd_entry.get()))

  def update_up(self, event=''):
    self.up_pub.publish(float(self.up_entry.get()))

  def update_lo(self, event=''):
    self.lo_pub.publish(float(self.lo_entry.get()))

  def update_rs(self, event=''):
    self.rs_pub.publish(float(self.rs_entry.get()))

# "main" code - sloppy but ok for now
rospy.init_node('tune_gui', anonymous=True)
root = Tk()
# show the namespace in the window title
root.wm_title(rospy.get_namespace())
app = App(root)
root.after(1000,app.check_ros)
root.mainloop()
root.destroy()
