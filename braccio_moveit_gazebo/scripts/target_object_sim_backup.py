#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import time

from gazebo_msgs.msg import LinkStates, ModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState

## END_SUB_TUTORIAL
import numpy as np
import scipy.optimize
import cv2
import json

THETA_EXT = 0.27
THETA_RET = np.pi/4

L_FUDGE = 0.08

Z_MAX_SIDE = -0.03
Z_MAX_DOWN = 0
Z_MIN = -0.045

CLOSE_ENOUGH = 0.02
DEFAULT_ROT = 0

S_SIDE_MAX = 0.4
S_SIDE_MIN = 0.161
S_TOP_MAX = 0.29

def cart2pol(x, y):
    """helper, convert cartesian to polar coordinates"""
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    """helper,convert polar to cartesian"""
    x = rho*np.cos(phi)
    y = rho*np.sin(phi)
    return(x, y)

def get_other_angles(theta_shoulder):
  """helper, converting some angles"""
  theta_wrist = theta_shoulder + np.pi/2
  theta_elbow = np.pi/2 - 2*theta_shoulder
  return theta_wrist, theta_elbow

class BraccioObjectTargetInterface(object):
  """BraccioXYBBTargetInterface"""
  def __init__(self):
    super(BraccioObjectTargetInterface, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('braccio_xy_bb_target', anonymous=True)

    group_name = "braccio_arm"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)
    self.gripper_group = moveit_commander.MoveGroupCommander("braccio_gripper")

    self.homography = None

    self.kinematics = Arm3Link()
    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.linkstate_callback)

  def linkstate_callback(self, data):
    """callback to get link location for cube from gazebo"""
    try:
      self.linkstate_data = data
    except ValueError:
      pass

  def reset_link(self, name, x, y, z):
    state_msg = ModelState()
    state_msg.model_name = name
    state_msg.pose.position.x = float(x)
    state_msg.pose.position.y = float(y)
    state_msg.pose.position.z = float(z)
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

  def reset_target_position(self):
    """reset block and bowl"""
    print 'reset block x='
    x = raw_input()
    print 'reset block y='
    y = raw_input()
    print 'reset block z='
    z = raw_input()
    self.reset_link('unit_box_0', x, y, z)
    self.reset_link('my_mesh', -0.15, -0.325, 0)

  def transform(self, x1, y1, r):
    """transform from gazebo coordinates into braccio coordinates"""
    if self.homography is not None:
      a = np.array([[x1, y1]], dtype='float32')
      res = cv2.perspectiveTransform(a[None, :, :], self.homography)[0][0]
      return float(res[0]), float(res[1]), DEFAULT_ROT
    else:
      raise ValueError('run or load calibration first!')

  def get_box_position(self):
    x, y, r = self.get_link_position(['unit_box_0::link'])
    return self.transform(x,y,r)

  def get_link_position(self, link_names):
    """get mean position of a list of links"""
    x = 0
    y = 0
    n = 0
    for l in link_names:
      ind = self.linkstate_data.name.index(l)
      res = self.linkstate_data.pose[ind].position
      x += res.x
      y += res.y
      n += 1
    return x/n, y/n, DEFAULT_ROT

  def calibrate(self):
    """scan a series of points and record points in gazebo and robot frames"""
    src_pts = []
    dst_angs = []
    mouseX, mouseY, r_ = self.get_link_position(['kuka::base_link'])
    src_pts.append([mouseX,mouseY])

    self.gripper_middle()
    N = 8
    phi_min = np.pi/6
    phi_max = np.pi - np.pi/6
    for i in range(2,N):
      self.go_to_raise()
      if i % 2 == 0:
        rand_phi = phi_min + i*(phi_max-phi_min)/N
        theta_shoulder = THETA_RET
      else:
        theta_shoulder = THETA_EXT
      theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
      rand_targ = [rand_phi,theta_shoulder,theta_elbow, theta_wrist]
      self.go_to_j(j0=rand_phi,j1=theta_shoulder,j2=theta_elbow,j3=theta_wrist)
      mouseX, mouseY, r_ = self.get_link_position(['kuka::left_gripper_link','kuka::right_gripper_link'])
      src_pts.append([mouseX,mouseY])
      dst_angs.append(rand_targ)
    with open('calibration.json', 'w') as f:
      json.dump({'src_pts':src_pts,'dst_angs':dst_angs},f)
    self.load_calibrate()
    self.go_to_up()

  def load_calibrate(self):
    """load mapping points from gazebo to robot frame, estimate l and L, generate homography map"""
    try:
      with open('calibration.json', 'r') as f:
        calib = json.load(f)
      src_pts = calib['src_pts']
      dst_angs = calib['dst_angs']

      s_ret_pts = src_pts[1::2]
      s_ext_pts = src_pts[2::2]
      arr = np.array(s_ret_pts)-np.array(s_ext_pts)
      self.L = np.sqrt((arr*arr).sum(axis=1)).mean()/(np.cos(THETA_EXT)-np.cos(THETA_RET))
      arr = np.array(s_ret_pts)-np.array(src_pts[0])
      l1 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_RET)
      arr = np.array(s_ext_pts)-np.array(src_pts[0])
      l2 = np.sqrt((arr*arr).sum(axis=1)).mean() - self.L*np.cos(THETA_EXT)
      self.l = (l1+l2)/2

      dst_pts = [[0,0]]
      for i in range(len(dst_angs)):
        phi = dst_angs[i][0]
        rho = self.L*np.cos(dst_angs[i][1]) + self.l
        x, y = pol2cart(rho, phi)
        dst_pts.append([x,y])

      src_pts = np.array(src_pts)
      dst_pts = np.array(dst_pts)

      h, status = cv2.findHomography(src_pts, dst_pts)
      self.homography = h

      self.kinematics = Arm3Link(L=[self.L/2,self.L/2,self.l+L_FUDGE])
      print 'calibration loaded.'
      print 'estimated l = ' + str(self.l)
      print 'estimated L = ' + str(self.L)
      cv2.destroyAllWindows()
    except:
      print 'calibration.json not in current directory, run calibration first'

  def go_to_j(self, j0=None, j1=None, j2=None, j3=None):
    """update arm joints"""
    joint_goal = self.move_group.get_current_joint_values()
    if j0 is not None:
      joint_goal[0]=j0
    if j1 is not None:
      joint_goal[1]=j1
    if j2 is not None:
      joint_goal[2]=j2
    if j3 is not None:
      joint_goal[3]=j3
    self.go_to_joint(joint_goal)

  def go_to_joint(self, joint_targets):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = joint_targets[0]
    joint_goal[1] = joint_targets[1]
    joint_goal[2] = joint_targets[2]
    joint_goal[3] = joint_targets[3]
    joint_goal[4] = 1.5708
    ret = self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()

  def gripper_close(self):
    self.go_gripper(1.2)

  def gripper_open(self):
    self.go_gripper(0.2)

  def gripper_middle(self):
    self.go_gripper(0.5)

  def go_gripper(self, val):
    joint_goal = self.gripper_group.get_current_joint_values()
    joint_goal[0] = val
    joint_goal[1] = val
    self.gripper_group.go(joint_goal, wait=True)
    self.gripper_group.stop()

  def go_to_raise(self):
    self.go_to_j(j1=1.15,j2=0.13,j3=2.29)

  def go_to_pull(self, phi):
    self.go_to_raise()
    self.gripper_close()
    if phi:
      self.go_to_j(j0=float(phi))
    self.go_to_j(j1=0.3,j2=1.8,j3=1.8)
    self.go_to_j(j1=0.3,j2=1.8,j3=0.1)
    self.go_to_j(j1=1.3,j2=0.4,j3=0.01)
    self.go_to_j(j1=1.5,j2=0.4,j3=0.1)
    self.go_to_j(j1=0.3,j2=1.8,j3=1.8)

  def go_to_push(self, phi):
    self.go_to_raise()
    self.gripper_close()
    if phi:
      self.go_to_j(j0=float(phi))
    self.go_to_j(j1=2.7,j2=0.01,j3=0.01)
    self.go_to_j(j1=1.6,j2=0.01,j3=0.01)
    self.go_to_j(j1=0.3,j2=1.8,j3=0.1)
    self.go_to_j(j1=2.1,j2=0.01,j3=0.01)
    self.go_to_j(j1=2.7,j2=0.01,j3=0.01)

  def get_targets(self,x,y):
    s, phi = cart2pol(x,y)
    q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_SIDE, 0)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print 'NO SOLUTION FOUND'
      print 'goal distance = '+str(s)
      print 'closest solution = '+str(xy[0])
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]

  def get_down_targets(self,x,y):
    s, phi = cart2pol(x,y)
    print s, phi
    q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_DOWN, -np.pi/2)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print 'NO SOLUTION FOUND'
      print 'goal distance = '+str(s)
      print 'closest solution = '+str(xy[0])
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]

  def go_to_xy(self, x, y, r, how):
    if how=='top':
      s, joint_targets = self.get_down_targets(x,y)
      print joint_targets
      if joint_targets[0]<0 or joint_targets[0]>3.14:
        print '++++++ Not in reachable area, aborting ++++++'
        return -1
      if np.isnan(joint_targets[1]) and s < S_TOP_MAX and s > S_SIDE_MIN:
        print '++++++ Too far out, pulling backwards +++++'
        self.go_to_pull(joint_targets[0])
        return 1
      if np.isnan(joint_targets[1]):
        print '++++++ Not in reachable area, aborting ++++++'
        return -1
    elif how=='side':
      s, joint_targets = self.get_targets(x,y)
      print joint_targets
      if joint_targets[0]<0 or joint_targets[0]>3.14:
        print '++++++ Not in reachable area, aborting ++++++'
        return -1
      if np.isnan(joint_targets[1]) and s < S_SIDE_MAX and s > S_SIDE_MIN:
        print '++++++ Too close, pushing backwards +++++'
        self.go_to_push(joint_targets[0])
        return 1
      if np.isnan(joint_targets[1]):
        print '++++++ Not in reachable area, aborting ++++++'
        return -1

    self.go_to_raise()
    self.gripper_open()
    self.go_to_j(j0=float(joint_targets[0]))

    self.go_to_j(j1=float(joint_targets[1]),
                 j2=float(joint_targets[2]),
                 j3=float(joint_targets[3]))

    self.gripper_close()
    if how=='top' and joint_targets[2]<3:
      self.go_to_j(j2=float(joint_targets[2])+0.1)
    self.go_to_home()
    return 0

  def go_to_manual_joint(self):
    joint_goal = self.move_group.get_current_joint_values()
    for i in range(len(joint_goal)):
      print 'joint' + str(i) + ' ' + str(joint_goal[i])
      tst = raw_input()
      if tst!='':
          joint_goal[i] = float(tst)
    self.go_to_joint(joint_goal)

  def go_to_manual(self, how):
    print 'pos x?'
    tst = raw_input()
    if tst!='':
        x = float(tst)
    print 'pos y?'
    tst = raw_input()
    if tst!='':
        y = float(tst)
    return self.go_to_xy(x, y, DEFAULT_ROT, how)

  def go_to_manual_gripper(self):
    print 'grip position?'
    tst = raw_input()
    if tst!='':
        v = float(tst)
    self.go_gripper(v)

  def go_to_target(self, how):
    x,y,r = self.get_box_position()
    print x, y, r
    return self.go_to_xy(x, y, r, how)

  def go_to_home(self):
    self.go_to_raise()
    self.go_to_j(j0=3.14)
    self.gripper_open()
    self.gripper_open()

  def go_to_bowl(self):
    self.go_to_up()
    self.gripper_open()
    self.go_to_j(j0=0.45,j1=1.57,j2=3.14,j3=3.14)
    self.go_to_j(j1=2.76,j2=2.82,j3=0.76)
    self.gripper_middle()
    self.go_to_j(j1=2.87,j2=2.52,j3=0.83)
    self.go_to_j(j1=2.5,j2=2.52,j3=0.83)
    self.go_to_j(j0=0.9)
    self.go_to_j(j1=2.87,j2=2.52,j3=0.83)
    self.gripper_open()
    self.go_to_j(j1=2.76,j2=2.82,j3=0.76)
    self.gripper_open()
    self.go_to_j(j1=1.57,j2=3.14,j3=3.14)
    self.go_to_up()


  def go_to_up(self):
    self.go_to_j(j0=1.5708,j1=1.5708,j2=1.5708,j3=1.5708)

  def run_eval(self):
    evl_data = []
    print "how many trials?"
    tst = raw_input()
    N = int(tst)
    for i in range(N):
      print "Running trial " + str(i)
      how = 'side' if np.random.uniform()<0.5 else 'top'
      extent = 0.5 if how=='side' else S_TOP_MAX
      x = np.random.uniform()*extent
      y = -extent + 2*extent*np.random.uniform()

      self.reset_link('unit_box_0', x, y, 0)
      self.reset_link('my_mesh', -0.15, -0.325, 0)

      time.sleep(1)

      record = {'target': [x,y], 'how': how}
      state = 1
      results = []
      for tries in range(3):
        x_, y_, r_ = self.get_link_position(['unit_box_0::link'])
        results.append([x_,y_,state])
        if state < 1:
          break
        state = self.go_to_target(how)
      if state==0:
        self.go_to_bowl()
        x_, y_, r_ = self.get_link_position(['unit_box_0::link'])
        results.append([x_,y_,state])

      record['box_results']=results
      x_, y_, r_ = self.get_link_position(['my_mesh::body'])
      record['bowl_result'] = [x_, y_]
      evl_data.append(record)
      with open('eval_results.json', 'w') as f:
        json.dump(evl_data,f)

class Arm3Link:
    """
    A simple inverse kinematics solver for a should-elbow-wrist robot arm
    credit: https://github.com/studywolf/blog/tree/master/InvKin
    """
    def __init__(self, L=None):
        # initial joint angles
        self.q = [0, 0, 0]
        # some default arm positions
        self.L = np.array([1, 1, 0.8]) if L is None else L
        self.max_y = 1
        self.min_y = 0

        self.end_angle_tol = 0.05
        self.end_angle = -np.pi/2
        self.max_angles = [1.6, np.pi/2, np.pi/2]
        self.min_angles = [0.27, -np.pi/2, -np.pi/2]

    def get_xy(self, q=None):
        if q is None:
            q = self.q

        x = self.L[0]*np.cos(q[0]) + \
            self.L[1]*np.cos(q[0]+q[1]) + \
            self.L[2]*np.cos(np.sum(q))

        y = self.L[0]*np.sin(q[0]) + \
            self.L[1]*np.sin(q[0]+q[1]) + \
            self.L[2]*np.sin(np.sum(q))

        return [x, y]

    def inv_kin(self, x, min_y, max_y, end_angle):

        def distance_to_default(q, x):
            x = (self.L[0]*np.cos(q[0]) + self.L[1]*np.cos(q[0]+q[1]) +
                 self.L[2]*np.cos(np.sum(q))) - x
            return x**2

        def y_upper_constraint(q, *args):
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q)))
            return self.max_y - y

        def y_lower_constraint(q, *args):
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q)))
            return y - self.min_y

        def joint_limits_upper_constraint(q, *args):
            return self.max_angles - q

        def joint_limits_lower_constraint(q, *args):
            return q - self.min_angles

        def joint_limits_last_orientation(q, *args):
            return self.end_angle_tol - np.abs(np.sum(q)-self.end_angle)

        self.min_y = min_y
        self.max_y = max_y
        if end_angle is not None:
            self.end_angle = end_angle
        q = scipy.optimize.fmin_slsqp(func=distance_to_default, x0=self.q, args=(x,), iprint=0,
                                      ieqcons=[joint_limits_last_orientation,
                                               joint_limits_upper_constraint,
                                               joint_limits_lower_constraint,
                                               y_upper_constraint,
                                               y_lower_constraint])
        self.q = q
        return self.q

def print_instructions():
  print ""
  print "==================== Instructions: ===================="
  print "c = calibrate, rerun calibration routine"
  print "t = target, pick up red block and drop on the ramp"
  print "m = manual, manually enter location for pickup"
  print "b = bowl, go through the preprogrammed bowl move"
  print "r = reset_target, set block to new location, reset bowl"
  print "e = evaluate, test pickup and collect statistics to file"
  print "q = quit program"
  print ""
  print "type next command:"

def main():
  print """
                _____  _____  _    _ _____ _   _  ____
          /\   |  __ \|  __ \| |  | |_   _| \ | |/ __ |
         /  \  | |__) | |  | | |  | | | | |  \| | |  | |
        / /\ \ |  _  /| |  | | |  | | | | | . ` | |  | |
       / ____ \| | \ \| |__| | |__| |_| |_| |\  | |__| |
      /_/    \_|_|  \_|_____/ \____/|_____|_| \_|\____/
        ____  _____           _____ _____ _____ ____
       |  _ \|  __ \    /\   / ____/ ____|_   _/ __ |
       | |_) | |__) |  /  \ | |   | |      | || |  | |
       |  _ <|  _  /  / /\ \| |   | |      | || |  | |
       | |_) | | \ \ / ____ | |___| |____ _| || |__| |
       |____/|_|  \_/_/    \_\_____\_____|_____\____/
    _____ _____ _____ _  __    _____  _____   ____  _____
   |  __ |_   _/ ____| |/ /_  |  __ \|  __ \ / __ \|  __ |
   | |__) || || |    | ' _| |_| |  | | |__) | |  | | |__) |
   |  ___/ | || |    |  |_   _| |  | |  _  /| |  | |  ___/
   | |    _| || |____| . \|_| | |__| | | \ \| |__| | |
   |_|   |_____\_____|_|\_\   |_____/|_|  \_\\____/|_|
   _____ _____ __  __ _    _ _            _______ ____  _____
  / ____|_   _|  \/  | |  | | |        /\|__   __/ __ \|  __ |
 | (___   | | | \  / | |  | | |       /  \  | | | |  | | |__) |
  \___ \  | | | |\/| | |  | | |      / /\ \ | | | |  | |  _  /
  ____) |_| |_| |  | | |__| | |____ / ____ \| | | |__| | | \ |
 |_____/|_____|_|  |_|\____/|______/_/    \_|_|  \____/|_|  \_\
"""
  print "Loading ...."
  bb_targetter = BraccioObjectTargetInterface()

  bb_targetter.load_calibrate()
  print ""
  print "++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

  print "  Welcome to the Arduino Braccio Pick+Drop Simulator!"
  print ""
  print "This is an example program for simulating control of"
  print "a Braccio arm using ROS and Gazebo physics simulator."
  print "++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

  while True:
      print_instructions()
      inp = raw_input()
      if inp=='q':
          break
      if inp=='c':
          bb_targetter.calibrate()
      if inp=='t':
        print 'pick from top (t) or side (s)?'
        inp2 = raw_input()
        if inp2 == 't':
          bb_targetter.go_to_target('top')
        if inp2 == 's':
          bb_targetter.go_to_target('side')
      if inp=='m':
        print 'pick from top (t) or side (s)?'
        inp2 = raw_input()
        if inp2 == 't':
          bb_targetter.go_to_manual('top')
        if inp2 == 's':
          bb_targetter.go_to_manual('side')
      if inp=='r':
          bb_targetter.reset_target_position()
      if inp=='b':
          bb_targetter.go_to_bowl()
      if inp=='e':
          bb_targetter.run_eval()
      if inp=='u':
          bb_targetter.go_to_up()


if __name__ == '__main__':
  main()
