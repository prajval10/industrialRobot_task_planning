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
from std_msgs.msg import String

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

##initialise lists

start_pose_rad = []
way1_rad = []    
way2_rad = []
grasp_app_rad = []
grasp_pose_rad = []
way3_rad = []
drop_app_rad =[]
drop_loc_rad =[]
start_pose = [-6.52, -83.81, -126.22, -46.29, 91.39, -1.78]
way1 = [-9.37,-62.96,-99.86,-93.42,92.09,-4.54]
way2 = [64.0,-73.29,-91.96,-98.70,77.44,67.91]
grasp_app = [64.0,-73.29,-91.96,-14.96,43.30,67.91]
grasp_pose = [55.38,-86.76,-91.98,-2.79,62.29,67.52]
way3 = [138.29,-86.76,-91.98,-2.79,62.29,67.52]
drop_app11 = [143.7,-112.77,-93.11,-62.60,87.15,86.39]
drop_loc11 = [143.7,-118.41,-98.12,-51.95,87.15,86.39]

##convert to radians
for it in range(len(start_pose)):
    start_pose_rad.append(math.radians(start_pose[it]))
    way1_rad.append(math.radians(way1[it]))
    way2_rad.append(math.radians(way2[it]))
    grasp_app_rad.append(math.radians(grasp_app[it]))
    grasp_pose_rad.append(math.radians(grasp_pose[it]))
    way3_rad.append(math.radians(way3[it]))
    drop_app_rad.append(math.radians(drop_app11[it]))
    drop_loc_rad.append(math.radians(drop_loc11[it]))
    
client = None

## Define list of actions in sequential order
actions = [start_pose_rad,way1_rad,way2_rad,grasp_app_rad,grasp_pose_rad,grasp_app_rad,way3_rad,drop_app_rad,drop_loc_rad,drop_app_rad,way3_rad,grasp_app_rad,way2_rad,way1_rad,start_pose_rad]


def move_grasp_place():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        for i in range(1,10):
            for x in actions:
                g.trajectory.points.append(JointTrajectoryPoint(positions= x, velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d+=2    
                pass
            pass   
        client.send_goal(g)
        client.wait_for_result()
        print client.get_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def callback(data):
    client.cancel_goal()
    pass
   
def main():
    global client
    try:
        rospy.init_node("pickPlace", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.Subscriber("/Obstacle", String , callback)
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

if __name__ == '__main__': 
    main()
