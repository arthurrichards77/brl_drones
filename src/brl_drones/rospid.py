import rospy
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

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

  def __init__(self, kp, ki, kd, namespace, lower_lim=-1.0, upper_lim=1.0):
    # constructor
    # namespace is location for tuning parameters

    # store own namespace
    self.namespace = rospy.resolve_name(namespace)
    rospy.loginfo('Setting up new PID controller in %s', self.namespace)

    # store default gains
    self.init_gains(kp, ki, kd)

    # store limits
    self.init_limits(lower_lim, upper_lim)

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
    self.sub_reset = rospy.Subscriber(self.namespace + "/reset_integrator", Float32, self.reset_int_callback)
    self.sub_upper = rospy.Subscriber(self.namespace + "/set_limits/upper", Float32, self.upper_lim_callback)
    self.sub_lower = rospy.Subscriber(self.namespace + "/set_limits/lower", Float32, self.lower_lim_callback)
    
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

  def init_limits(self, lower_lim, upper_lim):
    # initialize gains from various sources

    # just start by using given ones
    self.upper_lim = upper_lim
    self.lower_lim = lower_lim
    rospy.loginfo('PID %s given limits [%f,%f]', self.namespace, self.lower_lim, self.upper_lim)

    # check for parameter overrides
    limit_override_flag=False
    if rospy.has_param(self.namespace + "/init_limits/upper"):
      self.upper_lim = rospy.get_param(self.namespace + "/init_limits/upper")
      limit_override_flag=True
    if rospy.has_param(self.namespace + "/init_limits/lower"):
      self.lower_lim = rospy.get_param(self.namespace + "/init_limits/lower")
      limit_override_flag=True
    # report override
    if limit_override_flag:
      rospy.logwarn('PID %s got limits from parameters [%f,%f]', self.namespace, self.lower_lim, self.upper_lim)
    assert(self.upper_lim>self.lower_lim)

  def update(self, y, r, t):
    # return new output given new inputs
    # y is measurement; r is reference; t is time
    
    # start with proportional gain
    u = self.kp*(r - y)

    # only use the I and D parts if valid stored data
    if self.has_run == True:

      # find time since last update
      delta_t = t - self.last_t

      # rest only makes sense if time has elapsed
      if delta_t>0:

        # optional integrator update
        if not self.freeze_integrator_flag:

          # add to integrator using trapezium rule
          self.integ = self.integ + self.ki*0.5*delta_t*(self.last_e + r-y)

	  # saturate
          self.integ = saturate2(self.integ,self.lower_lim,self.upper_lim)

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

    # saturate
    u = saturate2(u,self.lower_lim,self.upper_lim)

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

  # callbacks for integrators and limits

  def reset_int_callback(self,data):
    self.reset_integrator(data.data)

  def upper_lim_callback(self, data):
    if data.data>self.lower_lim:
      self.upper_lim = data.data
      rospy.logwarn('PID %s updated upper limit: %f', self.namespace, self.upper_lim)
    else:
      rospy.logwarn('PID %s ignoring upper limit', self.namespace)

  def lower_lim_callback(self, data):
    if data.data<self.upper_lim:
      self.lower_lim = data.data
      rospy.logwarn('PID %s updated lower limit: %f', self.namespace, self.lower_lim)
    else:
      rospy.logwarn('PID %s ignoring lower limit', self.namespace)


class Quadpid:

  def __init__(self, tf_meas_frame, tf_targ_frame, ctrl_topic='cmd_vel'):
    # frame names
    self.tf_meas_frame = tf_meas_frame
    self.tf_targ_frame = tf_targ_frame
    # and the listener
    self.listener = tf.TransformListener()
    self.tf_ok = False
    # channels
    self.channel_names = ['pitch','roll','yaw','height']
    # four independent PID channels
    self.pids = [Rospid(0.0, 0.0, 0.0, '~'+ch) for ch in self.channel_names]
    # default reference values always zero
    self.refs = [0.0, 0.0, 0.0, 0.0]
    # prepare output channel - override for different formats
    self.setup_output(ctrl_topic)

  def setup_output(self,ctrl_topic):
    self.twist_pub = rospy.Publisher(ctrl_topic, Twist, queue_size=1)

  def update(self,t):
    # derive errors somehow - override this for different approaches
    self.calcRefsAndMeas()
    if self.tf_ok:
      # now update each of the channel PIDs
      self.ctrls = [self.pids[ii].update(self.meas[ii], self.refs[ii], t) for ii in range(4)]
      # and finally transmit the output - again, override for different quads
      self.pub_output()

  def calcRefsAndMeas(self):
    # flag in case of problems
    self.tf_ok = False
    # grab the transform
    try:
      [trans,rot] = self.listener.lookupTransform(self.tf_meas_frame, self.tf_targ_frame, rospy.Time(0))
      #print trans
      #print rot
      self.meas = [-trans[0], -trans[1], -rot[2], -trans[2]]
      rospy.loginfo('Measurement PRYZ relative to target is [%f %f %f %f]',-trans[0], -trans[1], -rot[2], -trans[2])
      self.tf_ok = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo('Problem with transform from %s to %s',self.tf_targ_frame,self.tf_meas_frame)

  def pub_output(self):
    # convert to a twist and transmit
    op = Twist()
    op.linear.x = self.ctrls[0]
    op.linear.y = self.ctrls[1]
    op.angular.z = self.ctrls[2]
    op.linear.z = self.ctrls[3]
    self.twist_pub.publish(op)

