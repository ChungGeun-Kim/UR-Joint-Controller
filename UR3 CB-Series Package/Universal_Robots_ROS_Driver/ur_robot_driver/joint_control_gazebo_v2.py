#!/usr/bin/env python

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import tf
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped
import numpy as np
from numpy import linalg
import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

global mat
mat=np.matrix


# ****** ur3(mm) Coefficients ******
global d1, a2, a3, d4, d5, d6, alph1, alph4, alph5
global d, a, alph
d1 =  0.1519
a2 = -0.24365
a3 = -0.21325
d4 =  0.11235
d5 =  0.08535
d6 =  0.0819
alph1 = pi/2
alph4 = pi/2
alph5 = -pi/2
d = mat([d1, 0, 0, d4, d5, d6])
a = mat([0, a2, a3, 0, 0, 0])
alph = mat([alph1, 0, 0, alph4, alph5, 0])

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

target_posi = [0,0,0,0,0,0]
S = [3.14, -1.57, 0., -1.57, -3.14, 0.]
client = None

# ************************************************** FORWARD KINEMATICS
def AH(n, th, c):

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
	           [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
	           [0,               0,              1, 0],
	           [0,               0,              0, 1]],copy=False)
      
  Rxa = mat([[1, 0,                 0,                  0],
	           [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
	           [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
	           [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
	    
  return A_i

def HTrans(pos, rad):
  r11 = cos(rad.x)*cos(rad.y)*cos(rad.z) - (sin(rad.x)*sin(rad.z))
  r12 = -(cos(rad.x)*cos(rad.y)*sin(rad.z)) - (sin(rad.x)*cos(rad.z))
  r13 = cos(rad.x)*sin(rad.y)
  r21 = sin(rad.x)*cos(rad.y)*cos(rad.z) + cos(rad.x)*sin(rad.z)
  r22 = -(sin(rad.x)*cos(rad.y)*sin(rad.z)) + cos(rad.x)*cos(rad.z)
  r23 = sin(rad.x)*sin(rad.y)
  r31 = -(sin(rad.y)*cos(rad.z))
  r32 = sin(rad.y)*sin(rad.z)
  r33 = cos(rad.z)

  forkin = mat([[r11, r12, r13, pos.x],
                [r21 ,r22, r23, pos.y],
                [r31, r32, r33, pos.z],
                [0,   0,   0,   1]],copy=False)

  return forkin

# ************************************************** INVERSE KINEMATICS 
def invKine(desired_pos):# T60
  th = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0, 0, -d6, 1]).T-mat([0, 0, 0, 1]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1, 0], P_05[1-1, 0])
  try:
    phi = acos(d4 / sqrt(P_05[2-1, 0] * P_05[2-1, 0] + P_05[1-1, 0] * P_05[1-1, 0]))
  except ValueError:
    return
  except ZeroDivisionError:
    return
  #The two solutions for theta1 correspond to the shoulder
  #being either left or right
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_16 = T_10 * desired_pos
    th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6)
    th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6)

  th = th.real
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_16 = linalg.inv( T_10 * desired_pos )
    th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
		  
  th = th.real

  # **** theta3 ****
  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_65 = AH( 6,th,c)
    T_54 = AH( 5,th,c)
    T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
    P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
    t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
    th[2, c] = t3.real
    th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c ))
    T_65 = linalg.inv(AH(6,th,c))
    T_54 = linalg.inv(AH(5,th,c))
    T_14 = (T_10 * desired_pos) * T_65 * T_54
    P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
	      
    # theta 2
    th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
    # theta 4
    T_32 = linalg.inv(AH( 3,th,c))
    T_21 = linalg.inv(AH( 2,th,c))
    T_34 = T_32 * T_21 * T_14
    th[3, c] = atan2(T_34[1,0], T_34[0,0])
  th = th.real

  return th

def move_start():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=S, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        ]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_target():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=target_posi, velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(0.05)),
    ]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def callback(data):
    global target_posi

    position = data.pose.position
    euler = data.pose.orientation

    print(data)
    forward_kin = HTrans(position, euler)
    inv_solution = invKine(forward_kin)
    try:
        target_posi = inv_solution.T[0]
    except AttributeError:
        return
    target_posi = target_posi.tolist()
    target_posi = target_posi[0]
    
    if target_posi[2] != 0.0:
        move_target()

def main():
    global client

    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        move_start()
        rospy.Subscriber('/point_axis', PoseStamped, callback)
        rospy.spin()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
