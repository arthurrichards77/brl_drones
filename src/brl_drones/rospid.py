import rospy
from std_msgs.msg import Float32

def saturate2(inp,lolimit,hilimit):
  # limit quantity to [lolimit,hilimit]
  if lolimit >= hilimit:
    rospy.logwarn('No room between limits [%f, %f]', lolimit, hilimit)
  out = inp
  if inp>hilimit:
    out=hilimit
    rospy.logwarn('Clamping %f at upper limit %f', inp, hilimit)
  elif inp<lolimit:
    out = lolimit
    rospy.logwarn('Clamping %f at lower limit %f', inp, lolimit)
  return out

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = saturate2(inp, -limit, limit)
  return out

class Rospid:

  def __init__(self, kp, ki, kd, namespace):
    # constructor
    # namespace is location for tuning parameters

    # store own namespace
    self.namespace = rospy.resolve_name(namespace)
    rospy.loginfo('Setting up new PID controller in %s', self.namespace)

    # store default gains
    self.init_gains(kp, ki, kd)

    # stores for last input values for differentiating
    self.last_t = 0.0
    self.last_y = 0.0
    # flag for whether or not stores have been initialized
    self.has_run = False

    # integrator
    self.integ = 0.0
    self.last_e = 0.0
    # option to disable integral action
    self.freeze_integrator_flag = False

    # create gain update subscribers
    self.sub_kp = rospy.Subscriber(self.namespace + "/tune_gains/kp", Float32, self.tune_kp_callback)
    self.sub_ki = rospy.Subscriber(self.namespace + "/tune_gains/ki", Float32, self.tune_ki_callback)
    self.sub_kd = rospy.Subscriber(self.namespace + "/tune_gains/kd", Float32, self.tune_kd_callback)
    
    # closing message
    rospy.loginfo('PID ready in %s', self.namespace)

  def init_gains(self, kp, ki, kd):
    # initialize gains from various sources

    # just start by using given ones
    self.kp = kp
    self.ki = ki
    self.kd = kd
    rospy.loginfo('PID %s given gains kp=%f, ki=%f, kd=%f', self.namespace, self.kp, self.ki, self.kd)

    # check for parameter overrides
    gain_override_flag=False
    # proportional
    if rospy.has_param(self.namespace + "/init_gains/kp"):
      self.kp = rospy.get_param(self.namespace + "/init_gains/kp")
      gain_override_flag=True
    # integral
    if rospy.has_param(self.namespace + "/init_gains/ki"):
      self.ki = rospy.get_param(self.namespace + "/init_gains/ki")
      gain_override_flag=True
    # derivative
    if rospy.has_param(self.namespace + "/init_gains/kd"):
      self.kd = rospy.get_param(self.namespace + "/init_gains/kd")
      gain_override_flag=True
    # report override
    if gain_override_flag:
      rospy.logwarn('PID %s got gains from parameters kp=%f, ki=%f, kd=%f', self.namespace, self.kp, self.ki, self.kd)

  def update(self, y, r, t):
    # return new output given new inputs
    # y is measurement; r is reference; t is time
    
    # start with proportional gain
    u = self.kp*(r - y)

    # only use the I and D parts if valid stored data
    if self.has_run == True:

      # find time since last update
      delta_t = t - self.last_t

      # print t, self.last_t, delta_t

      # optional integrator update
      if not self.freeze_integrator_flag:

        # add to integrator using trapezium rule
        self.integ = self.integ + self.ki*0.5*delta_t*(self.last_e + r-y)

      # add integral term to control
      u = u + self.integ        

      # find derivatives using finite difference
      dydt = (y - self.last_y)/delta_t

      # add to control - note negative
      u = u - self.kd*dydt

    # stores for future use
    self.last_e = r-y
    self.last_y = y
    self.last_t = t
    self.has_run = True

    # return the new control value
    return u

  # callbacks for online tuning

  def tune_kp_callback(self, data):
    # get new gains from ROS parameters
    self.kp = data.data
    rospy.logwarn('PID %s updated kp: %f', self.namespace, self.kp)

  def tune_ki_callback(self, data):
    # get new gains from ROS parameters
    self.ki = data.data
    rospy.logwarn('PID %s updated ki: %f', self.namespace, self.ki)

  def tune_kd_callback(self, data):
    # get new gains from ROS parameters
    self.kd = data.data
    rospy.logwarn('PID %s updated kd: %f', self.namespace, self.kd)

  # utilities for manually resetting and freezing integrator

  def reset_integrator(self, new_value):
    # reset integrator value
    self.integ = new_value
    rospy.logwarn('PID %s reset integrator to %f', self.namespace, self.integ)

  def freeze_integrator(self):
    if not self.freeze_integrator_flag:
      rospy.logwarn('PID %s frozen integrator at %f', self.namespace, self.integ)
    self.freeze_integrator_flag = True

  def enable_integrator(self):
    if self.freeze_integrator_flag:
      rospy.logwarn('PID %s enabled integrator at %f', self.namespace, self.integ)
    self.freeze_integrator_flag = False

  def read_integrator(self):
    return(self.integ)
