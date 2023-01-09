#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

S = [3.14, -1.57, 0., -1.57, -3.14, 0.]
target_pose = []
target_velo = []
client = None
time_now = 0
start_time = time.time()
base_control = []
base_robot = []
shoulder_control = []
shoulder_robot = []
elbow_control = []
elbow_robot = []
wrist1_control = []
wrist1_robot = []
wrist2_control = []
wrist2_robot = []
wrist3_control = []
wrist3_robot = []
time_check = []

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
        JointTrajectoryPoint(positions=target_pose, velocities=target_velo, time_from_start=rospy.Duration(0.05)),
    ]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def callback(data):
    global target_pose, target_velo, time_now
    time_now = time.time()
    joint_value = data
    target_pose = joint_value.position
    target_velo = joint_value.velocity
    move_target()
    """
    joint_states = rospy.wait_for_message("joint_states", JointState)
    base_control.append(round(joint_value.position[0], 4))
    base_robot.append(round(joint_states.position[2], 4))
    shoulder_control.append(round(joint_value.position[1], 4))
    shoulder_robot.append(round(joint_states.position[1], 4))
    elbow_control.append(round(joint_value.position[2], 4))
    elbow_robot.append(round(joint_states.position[0], 4))
    wrist1_control.append(round(joint_value.position[3], 4))
    wrist1_robot.append(round(joint_states.position[3], 4))
    wrist2_control.append(round(joint_value.position[4], 4))
    wrist2_robot.append(round(joint_states.position[4], 4))
    wrist3_control.append(round(joint_value.position[5], 4))
    wrist3_robot.append(round(joint_states.position[5], 4))
    time_check.append(round(time_now - start_time, 4))   
    """
def main():
    global client,base_control, base_robot, shoulder_control, shoulder_robot
    global elbow_control, elbow_robot, wrist1_control, wrist1_robot
    global wrist1_control, wrist1_robot, wrist2_control, wrist2_robot
    global time_check, wrist3_control, wrist3_robot

    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        move_start()
        rospy.Subscriber('/init_position', JointState, callback)
        rospy.spin()
        """
        for i in range(len(base_control)):
            base_control[i] = float(base_control[i])
            base_robot[i] = float(base_robot[i])
            shoulder_control[i] = float(shoulder_control[i])
            shoulder_robot[i] = float(shoulder_robot[i])
            elbow_control[i] = float(elbow_control[i])
            elbow_robot[i] = float(elbow_robot[i])
            wrist1_control[i] = float(wrist1_control[i])
            wrist1_robot[i] = float(wrist1_robot[i])
            wrist2_control[i] = float(wrist2_control[i])
            wrist2_robot[i] = float(wrist2_robot[i])
            wrist3_control[i] = float(wrist3_control[i])
            wrist3_robot[i] = float(wrist3_robot[i])
        
        ax=plt.subplot(1, 1, 1)
        ax.grid(True, axis = 'x')
        ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
        ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.1))
        plt.ylim([-3.2, 3.2])
        plt.plot(time_check, base_control, label='Theta_1')
        plt.plot(time_check, base_robot, label='Base', linestyle='--')
        plt.plot(time_check, shoulder_control, label='Theta_2')
        plt.plot(time_check, shoulder_robot, label='Shoulder', linestyle='--')
        plt.plot(time_check, elbow_control, label='Theta_3')
        plt.plot(time_check, elbow_robot, label='Elbow', linestyle='--')
        plt.plot(time_check, wrist1_control, label='Theta_4')
        plt.plot(time_check, wrist1_robot, label='Wrist1', linestyle='--')
        plt.plot(time_check, wrist2_control, label='Theta_5')
        plt.plot(time_check, wrist2_robot, label='Wrist2', linestyle='--')
        plt.plot(time_check, wrist3_control, label='Theta_6')
        plt.plot(time_check, wrist3_robot, label='Wrist3', linestyle='--')
        plt.xlabel('Time', fontsize=15)
        plt.ylabel('Deg', fontsize=15)
        plt.legend(loc = 2, fontsize=15)
        plt.show()
        """
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
