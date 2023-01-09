#!/usr/bin/env python

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import tf
import message_filters
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from ur_ikfast import ur_kinematics
import numpy as np

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

ur3_arm = ur_kinematics.URKinematics('ur3')

S = [0.0, -0.78, -1.56, -0.78, -1.56, 0.]
client = None
pos = None
pre = PoseStamped().pose.position
pre.x = 1.08604155e-01
pre.y = 1.13234207e-01
pre.z = 5.60028553e-01
ori = None
img = None
target_posi = None

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
        JointTrajectoryPoint(positions=target_posi, velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def callback(data):
    global img
    img = data.linear

def callback_1(data):
    global pos, ori, pre
    pos = data.pose.position
    ori = data.pose.orientation
    print(pre)
    print("+++++++")
    if (pos.x != 0.0) and (pos.y != 0.0) and (pos.z != 0.0):
        pre = pos

def callback_2(data):
    global pos, ori, target_posi, img, pre
    joints = data.position
    ur3_arm.forward(joints)
    if pos is not None and ori is not None:
        result = ur3_arm.inverse(np.array([-pre.x+img.x, pre.z-img.z, pre.y-img.y, ori.x, ori.y, ori.z, ori.w]), False, q_guess=joints)
        if result is not None:
            target_posi = [result[0], result[1], result[2], result[3], result[4], result[5]]
            move_target()

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        move_start()    
        rospy.Subscriber('/position', Twist, callback)
        rospy.Subscriber('/point_axis', PoseStamped, callback_1)
        rospy.Subscriber('/joint_states', JointState, callback_2)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
