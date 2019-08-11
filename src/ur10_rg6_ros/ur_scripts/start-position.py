#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import math
import numpy as np

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

##initialise lists

start_pose_rad = []
way1_rad = []    
way2_rad = []
grasp_app_rad = []
grasp_pose_rad = []
way3_rad = []

start_pose = [-6.52, -83.81, -126.22, -46.29, 91.39, -1.78]

##convert to radians
for it in range(len(start_pose)):
    start_pose_rad.append(math.radians(start_pose[it]))


client = None

def move_grasp_place():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        g.trajectory.points.append(JointTrajectoryPoint(positions=start_pose_rad, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        client.send_goal(g)
        client.wait_for_result()
        print client.get_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

   
def main():
    global client

    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        move_grasp_place()
        
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
