#!/usr/bin/python
from Tkinter import *

import roslib
roslib.load_manifest('brl_drones')
import sys
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Empty

class DronePanel:

  def __init__(self, master, title="Drone Control Panel"):
    frame=Frame(master, bg="yellow")
    master.title(title)
    frame.pack()
    self.my_frame = frame
    self.setup_ros_pubs()
    self.setup_ros_subs()
    self.setup_vel_buttons()
    self.setup_quit_button()

  def setup_quit_button(self):
    # quit button
    self.quit_button = Button(self.my_frame, text="Quit", command=self.my_frame.quit)
    self.quit_button.grid(row=0, column=4)

  def vel_btn_callback(self,vel):
    self.vel_pub.publish(vel)
    self.stop_auto()

  def vel_button(self,row,column,btn_vel,btn_text,bg="gray"):
    btn_callback = lambda: self.vel_btn_callback(btn_vel)
    btn = Button(self.my_frame, text=btn_text, command=btn_callback, bg=bg)
    btn.grid(row=row, column=column, padx=10, pady=10)
    return(btn)

  def setup_vel_buttons(self):
    # standard velocity messages
    self.up_vel = Twist()
    self.up_vel.linear.z = 0.1
    self.up = self.vel_button(1,3,self.up_vel,'Up')

    self.dn_vel = Twist()
    self.dn_vel.linear.z = -0.1
    self.dn = self.vel_button(3,3,self.dn_vel,'Down')

    self.fw_vel = Twist()
    self.fw_vel.linear.x = 0.1
    self.fw = self.vel_button(1,1,self.fw_vel,'FWD')

    self.rv_vel = Twist()
    self.rv_vel.linear.x = -0.1
    self.rv = self.vel_button(3,1,self.rv_vel,'BACK')

    self.lf_vel = Twist()
    self.lf_vel.linear.y = 0.1
    self.lf = self.vel_button(2,0,self.lf_vel,'LEFT')

    self.rt_vel = Twist()
    self.rt_vel.linear.y = -0.1
    self.rt = self.vel_button(2,2,self.rt_vel,'RIGHT')

    # yaw messages
    self.yaw_l = Twist()
    self.yaw_l.angular.z = 0.1
    self.yl = self.vel_button(1,0,self.yaw_l,'L TURN')

    self.yaw_r = Twist()
    self.yaw_r.angular.z = -0.1
    self.yr = self.vel_button(1,2,self.yaw_r,'R TURN')

    # stop buttons
    self.stop_vel = Twist()
    self.stop1 = self.vel_button(2,1,self.stop_vel,'STOP',bg="red")
    self.stop1 = self.vel_button(2,3,self.stop_vel,'STOP',bg="red")

    # landing and takeoff
    self.land = Button(self.my_frame, text='Land', bg="red", command=self.land_callback)
    self.land.grid(row=4, column=2, padx=10, pady=10)

    self.takeoff = Button(self.my_frame, text='Take off', bg="green", command=self.takeoff_callback)
    self.takeoff.grid(row=4, column=0, padx=10, pady=10)

    self.rst = Button(self.my_frame, text='Reset', bg="blue", fg="white", command=self.reset_callback)
    self.rst.grid(row=4, column=1, padx=10, pady=10)

    # external control
    self.auto_var = IntVar()
    self.aut = Checkbutton(self.my_frame, text="External", variable=self.auto_var)
    self.aut.grid(row=3, column=4, padx=10, pady=10)

    # repeaters for external input
    self.ext_x = Entry(self.my_frame)
    self.ext_x.insert(0,"External")
    self.ext_x.grid(row=4,column=4, padx=10, pady=10)

    # mode status message
    self.mode_msg = Entry(self.my_frame)
    self.mode_msg.insert(0,"Mode")
    self.mode_msg.grid(row=2,column=4, padx=10, pady=10)

  def takeoff_callback(self):
    self.takeoff_pub.publish(Empty())

  def land_callback(self):
    self.land_pub.publish(Empty())
    self.stop_auto()

  def reset_callback(self):
    self.reset_pub.publish(Empty())
    self.stop_auto()

  def setup_ros_pubs(self):
    self.msg_pub = rospy.Publisher('monitor/status_msg',String,queue_size=1)
    self.vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    self.land_pub = rospy.Publisher('ardrone/land', Empty,queue_size=1)
    self.reset_pub = rospy.Publisher('ardrone/reset', Empty,queue_size=1)
    self.takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty,queue_size=1)

  def setup_ros_subs(self):
    self.ext_sub = rospy.Subscriber('ext_vel', Twist, self.ext_callback)

  def ext_callback(self,data):
    self.ext_x.delete(0,END)
    self.ext_x.insert(0,"%.2f %.2f %.2f %.2f" % (data.linear.x,data.linear.y,data.linear.z,data.angular.z))
    if self.auto_var.get()==1:
      self.vel_pub.publish(data)

  def stop_btn(self):
    rospy.loginfo("Stop button pressed")
    # send zero velocity
    self.vel_pub.publish(Twist())
    self.stop_auto()

  def stop_auto(self):
    self.aut.deselect()

  def check_ros(self):
    if rospy.is_shutdown():
      self.my_frame.quit()
    else:
      root.after(1000,app.check_ros)

rospy.init_node('control_panel', anonymous=True)

drone_name=rospy.get_param('drone_name','Drone')
panel_title=drone_name + " Control Panel"

root = Tk()
app = DronePanel(root, title=panel_title)
root.after(1000,app.check_ros)
root.mainloop()
root.destroy()
