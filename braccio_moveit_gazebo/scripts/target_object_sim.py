#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import time
import math
import copy
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

def degrees_to_radians(degrees):
    return degrees * (math.pi / 180)


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

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

  def reset_target_position(self):
    """reset block and bowl"""
    print('reset block x=')

    x = input()
    #print 'reset block y='
    print('reset block y=')
    y = input()
    print('reset block z=')
    #print 'reset block z='
    z = input()
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
      print ('calibration loaded.')
      print ('estimated l = ' + str(self.l))
      print ('estimated L = ' + str(self.L))
      cv2.destroyAllWindows()
    except:
      print ('calibration.json not in current directory, run calibration first')

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
    q = self.kinematics.inv_kin(s, Z_MIN,Z_MAX_SIDE, 0)
    print("inv_kin found value",q)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print ('NO SOLUTION FOUND')
      print ('goal distance = '+str(s))
      print ('closest solution = '+str(xy[0]))
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]


  def get_draw_targets(self,x,y):
    s, phi = cart2pol(x,y)
    print("Cart2Pol Angle:",phi,"Distance:",s)
    q = self.kinematics.inv_kin(s, -0.05, 0.05, 0)
    #q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_SIDE, 0)
    #q = self.kinematics.inv_kin(s, 0, 5, 0)
    print("inv_kin found value",q)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print ('NO SOLUTION FOUND')
      print ('goal distance = '+str(s))
      print ('closest solution = '+str(xy[0]))
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]



  def get_down_targets(self,x,y):
    s, phi = cart2pol(x,y)
    print (s, phi)
    q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_DOWN, -np.pi/2)
    print(q)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print ('NO SOLUTION FOUND')
      print ('goal distance = '+str(s))
      print ('closest solution = '+str(xy[0]))
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]

  def go_to_xy(self, x, y, r, how):
    if how=='top':
      s, joint_targets = self.get_down_targets(x,y)
      print (joint_targets)
      if joint_targets[0]<0 or joint_targets[0]>3.14:
        print ('++++++ Not in reachable area, aborting ++++++')
        return -1
      if np.isnan(joint_targets[1]) and s < S_TOP_MAX and s > S_SIDE_MIN:
        print ('++++++ Too far out, pulling backwards +++++')
        self.go_to_pull(joint_targets[0])
        return 1
      if np.isnan(joint_targets[1]):
        print ('++++++ Not in reachable area, aborting ++++++')
        return -1
    elif how=='side':
      s, joint_targets = self.get_targets(x,y)
      print (joint_targets)
      if joint_targets[0]<0 or joint_targets[0]>3.14:
        print ('++++++ Not in reachable area, aborting ++++++')
        return -1
      if np.isnan(joint_targets[1]) and s < S_SIDE_MAX and s > S_SIDE_MIN:
        print ('++++++ Too close, pushing backwards +++++')
        self.go_to_push(joint_targets[0])
        return 1
      if np.isnan(joint_targets[1]):
        print ('++++++ Not in reachable area, aborting ++++++')
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
      print ('joint' + str(i) + ' ' + str(joint_goal[i]))
      tst = input()
      if tst!='':
          joint_goal[i] = float(tst)
    self.go_to_joint(joint_goal)

  def go_to_joint_position(self):
    print ('Phi?')
    tst = input()
    if tst!='':
        x = float(tst)
    print ('Shoulder?')
    tst = input()
    if tst!='':
        y = float(tst)
    print ('Elbow?')
    tst = input()
    if tst!='':
        z = float(tst)
    print ('Wrist?')
    tst = input()
    if tst!='':
        a = float(tst)

    self.go_to_j(x,y,z,a)

 
  def regular_pentagon_points(self):
    # Vertices of the regular pentagon

    vertices = [[-1, 0]]  # Vertex at (-1, 0)

    # Generate other vertices in a counter-clockwise manner
    for i in range(1, 5):
        angle = (i * 2 * np.pi / 5) + np.pi  # Shifted by -180 degrees to start from (-1, 0)
        x = np.cos(angle)
        y = np.sin(angle)
        vertices.append([x, y])

    print("****",vertices)
    # Number of equidistant points on each side (excluding the vertices)
    num_points = 10
    
    # Initialize list to store coordinates of equidistant points
    equidistant_points = []
    
    # Compute equidistant points on each side of the pentagon
    for i in range(5):
        start_point = vertices[i]
        end_point = vertices[(i + 1) % 5]
        
        for j in range(num_points):
            t = (j + 1) / (num_points + 1)  # Parameter for interpolation
            x = (1 - t) * start_point[0] + t * end_point[0]
            y = (1 - t) * start_point[1] + t * end_point[1]
            equidistant_points.append([x, y])
    
        equidistant_points.append(end_point)
    equidistant_points.append(vertices[i])
    
    return equidistant_points


  def equilateral_triangle_points(self):
        # Vertices of the equilateral triangle

        #print("*************",cart2pol(1,0))
        
        B = np.array([np.sqrt(3)/6,-0.5])
        C = np.array([np.sqrt(3)/6,0.5])
        A = np.array([-np.sqrt(3)/3,0])

        # Number of equidistant points on each side (excluding the vertices)
        num_points = 10

        # Initialize list to store coordinates of equidistant points
        equidistant_points = []

        # Compute equidistant points on side AB
        for i in range(num_points):
            t = (i + 1) / (num_points + 1)  # Parameter for interpolation
            P = (1 - t) * A + t * B
            equidistant_points.append(P)

        equidistant_points.append(B)
        # Compute equidistant points on side BC
        for i in range(num_points):
            t = (i + 1) / (num_points + 1)  # Parameter for interpolation
            P = (1 - t) * B + t * C
            equidistant_points.append(P)

        equidistant_points.append(C)
        # Compute equidistant points on side CA
        for i in range(num_points):
            t = (i + 1) / (num_points + 1)  # Parameter for interpolation
            P = (1 - t) * C + t * A
            equidistant_points.append(P)

        equidistant_points.append(A)

        return equidistant_points


  def trace_pentagon(self):
        self.go_to_up()
        points = self.regular_pentagon_points()
        for point in points:
            # Assuming implementation of get_draw_targets method
            s, joint_targets = self.get_draw_targets(point[0]/2.5,point[1]/2.5)
            #s, joint_targets = self.get_draw_targets(point[0]/1.5,point[1]/1.5)
            print(joint_targets)
            # Assuming implementation of go_to_j method
            self.go_to_j(j0=float(joint_targets[0]), j1=float(joint_targets[1]),
                             j2=float(joint_targets[2]), j3=float(joint_targets[3]))



  def trace_triangle(self):
        self.go_to_up()
        #points = self.regular_pentagon_points()
        points = self.equilateral_triangle_points()

        for point in points:
            # Assuming implementation of get_draw_targets method
            s, joint_targets = self.get_draw_targets(point[0]/2.5,point[1]/2.5)
            #s, joint_targets = self.get_draw_targets(point[0]/1.5,point[1]/1.5)
            print(joint_targets)
            # Assuming implementation of go_to_j method
            self.go_to_j(j0=float(joint_targets[0]), j1=float(joint_targets[1]),
                             j2=float(joint_targets[2]), j3=float(joint_targets[3]))


  def trace_square(self):
      #self.go_to_up()
      pi4 = math.pi/4
      pi2 = math.pi/2

      #coordinates in on direction
      coordinates = [
              (0,1),
              (pi4/4,1.02),
              (pi4/2,1.08),
              (3*pi4/4,1.202),
              (pi4,1.414),
              (5*pi4/4,1.202),
              (3*pi4/2,1.08),
              (7*pi4/4,1.02)
              ]

      values = [-2,-1,0,1]
      for i in values:
        for x in coordinates:
            s, joint_targets = self.get_draw_targets(0,x[1]/3.4)
            print(joint_targets)
            self.go_to_j(j0=float(x[0]+i*pi2),j1=float(joint_targets[1]),
                 j2=float(joint_targets[2]),
                 j3=float(joint_targets[3]))



            #self.go_to_j(joint_targets[0],joint_targets[1],joint_targets[2],joint_targets[3])
            #self.go_to_j(j0=float(x[0]))


  def trace_square_old1(self):
      #Sunny square function
      start_joint_angles = (0.0, math.pi/2, 0, 0)
      #corner_angles = [
      #  (0, 0, -90, 0),    # Move right
      #  (0, 0, -90, 0),    # Move down
      #  (0, 0, -90, 0),    # Move left
      #  (0, 0, -90, 0)     # Move up
      #  ]

     
      #corner_angles = [
      #  (0, math.pi / 2, math.pi / 2, 0),      # Move right
      #  (math.pi / 2, math.pi / 2, 0, 0),       # Move down
      #  (math.pi, math.pi / 2, math.pi / 2, 0), # Move left
      #  (math.pi / 2, math.pi / 2, 0, 0)    # Move up
      # ]

      #corner1 = (0, np.pi/2, np.pi/2, np.pi/2)  # Base, Shoulder, Elbow, Wrist
      #corner2 = (0, np.pi/2, 0, np.pi/2)
      #corner3 = (0, np.pi/2, 0, 0)
      #corner4 = (0, np.pi/2, np.pi/2, 0)

      #self.go_to_j(*corner1)
      #self.go_to_j(*corner2)
      #self.go_to_j(*corner3)
      #self.go_to_j(*corner4)

      #self.go_to_j(j0=0.0,j1=math.pi/2,j2=0.0,j3=0.0)
      #self.go_to_j(*start_joint_angles)

      #for corner in corner_angles:
      #    self.go_to_j(*corner)

      #x = -10.0/80.0
      #y = -10.0/80.0
      #s, joint_targets = self.get_draw_targets(x,y)
      #ret = self.move_group.go(joint_targets, wait=True)
      #print("Targets:",joint_targets)
      #self.go_to_j(j0=float(4.0))
      #self.go_to_j(j0=float(joint_targets[0]))
      #self.go_to_j(j1=float(joint_targets[1]),j2=float(joint_targets[2]),j3=float(joint_targets[3]))
      #self.gripper_open()



      #x = 10.0/80.0
      #y = 10.0/80.0
      #s, joint_targets = self.get_draw_targets(x,y)
      #ret = self.move_group.go(joint_targets, wait=True)
      #self.move_group.stop()
      #print("Targets:",joint_targets)
      #self.go_to_j(j0=float(joint_targets[0]))
      #self.go_to_j(j1=float(joint_targets[1]),j2=float(joint_targets[2]),j3=float(joint_targets[3]))
      #self.gripper_open()

      #x = -10.0/80.0
      #y = 10.0/80.0
      #s, joint_targets = self.get_draw_targets(x,y)
      #ret = self.move_group.go(joint_targets, wait=True)
      #self.move_group.stop()
      #print("Targets:",joint_targets)
      #self.go_to_j(j0=float(joint_targets[0]))
      #self.go_to_j(j1=float(joint_targets[1]),j2=float(joint_targets[2]),j3=float(joint_targets[3]))
      #self.gripper_open()
      #self.go_to_j(joint_targets[0],joint_targets[1],joint_targets[2],joint_targets[3])


    
      #self.move_group.stop()
      #self.go_to_j(joint_targets[0],joint_targets[1],joint_targets[2],joint_targets[3])

      #self.go_to_j(joint_targets[0],joint_targets[1],joint_targets[2],joint_targets[3])
      #print(joint_targets)
      #return


      
      for x in range(-10,10,1):
        for y in range(-10,10,1):
            print("Before: ",x,y)
            a = x/80.0
            b = y/80.0
            print("After: ",a,b)
            s, joint_targets = self.get_draw_targets(a,b)
            print(joint_targets)
            #self.go_to_j(joint_targets[0],joint_targets[1],joint_targets[2],joint_targets[3])
            self.go_to_j(j0=float(joint_targets[0]))
            self.go_to_j(j1=float(joint_targets[1]),
                 j2=float(joint_targets[2]),
                 j3=float(joint_targets[3]))
      #      break
        break


  def trace_square_old(self):
      #Sunny square function
      start_joint_angles = (0.0, math.pi/2, 0, 0)
      scale = 2.0

      waypoints = []
      wpose = self.move_group.get_current_pose().pose
      print(wpose)
      wpose.position.z = 0.125 # First move up (z)
      wpose.position.y = 0.125  # and sideways (y)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x += scale * 0.4  # Second move forward/backwards in (x)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.y -= scale * 0.4  # Third move sideways (y)
      waypoints.append(copy.deepcopy(wpose))
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
      print(plan,fraction)
      self.move_group.execute(plan,wait=True)
      planning_frame = self.move_group.get_planning_frame()
      print("============ Planning frame: %s" % planning_frame)
      



  def go_to_manual(self, how):
    print ('pos x?')
    tst = input()
    if tst!='':
        x = float(tst)
    print ('pos y?')
    tst = input()
    if tst!='':
        y = float(tst)
    return self.go_to_xy(x, y, DEFAULT_ROT, how)

  def go_to_manual_gripper(self):
    print ('grip position?')
    tst = input()
    if tst!='':
        v = float(tst)
    self.go_gripper(v)

  def go_to_target(self, how):
    x,y,r = self.get_box_position()
    print (x, y, r)
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
    print ("how many trials?")
    tst = input()
    N = int(tst)
    for i in range(N):
      print ("Running trial " + str(i))
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


    def inv_kin(self, x, min_y, max_y, end_angle,start=[0,0,0]):

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
        self.q = start
        print("Optimizer input: ",x)
        q = scipy.optimize.fmin_slsqp(func=distance_to_default, x0=self.q, args=(x,), iprint=0,
                                      ieqcons=[#joint_limits_last_orientation,
                                               joint_limits_upper_constraint,
                                               joint_limits_lower_constraint,
                                               y_upper_constraint,
                                               y_lower_constraint],iter=10000)
        self.q = q
        return self.q











def print_instructions():
  print ("")
  print ("==================== Instructions: ====================")
  print ("c = calibrate, rerun calibration routine")
  print ("j = go to a position")
  print ("s = trace a square")
  print ("p = trace a pentagon")
  print ("t = trace a triangle")
  print ("q = quit program")
  print ("")
  print ("type next command:")

def main():
  print ("""
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
   _____ _____ __  __ _    _ _            _______ ____  _____
  / ____|_   _|  \/  | |  | | |        /\|__   __/ __ \|  __ |
 | (___   | | | \  / | |  | | |       /  \  | | | |  | | |__) |
  \___ \  | | | |\/| | |  | | |      / /\ \ | | | |  | |  _  /
  ____) |_| |_| |  | | |__| | |____ / ____ \| | | |__| | | \ |
 |_____/|_____|_|  |_|\____/|______/_/    \_|_|  \____/|_|  \_\
""")
  print ("Loading ....")
  bb_targetter = BraccioObjectTargetInterface()

  bb_targetter.load_calibrate()
  print ("")
  print ("++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

  print ("  Welcome to the Arduino Braccio Shape Tracer!  ")
  print ("")
  print ("This is an example program for simulating control of")
  print ("a Braccio arm using ROS and Gazebo physics simulator.")
  print ("++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

  while True:
      print_instructions()
      inp = input()
      if inp=='q':
          break
      if inp=='c':
          bb_targetter.calibrate()
          bb_targetter.go_to_manual('top')
      if inp=='j':
          bb_targetter.go_to_joint_position()
      if inp=='s':
          bb_targetter.trace_square()
      if inp=='t':
          bb_targetter.trace_triangle()
      if inp=='p':
          bb_targetter.trace_pentagon()


if __name__ == '__main__':
  main()
