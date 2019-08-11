#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
import math
from std_msgs.msg import String

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
grasp_loc_rad = []
goal_loc_rad = []    
grasp_loc =[124.95, -92.35, -147.90,-25.29 , 82.59, -50.17]
goal_loc = [161.91, -92.35, -147.90,-25.29 , 82.59, -50.17]

startPose_rad = []
approachObj_rad = []
intGoal_rad = []
approachGoal_rad =[]
goalPose1_rad = []
startPose =[21.23, -110.47, -97.20, -63.65, 85.98, 79.49]
approachObj =[23.03, -124.60, -75.35, -71.45, 85.85, 81.83]
intGoal =[66.57, -78.27, -132.10, -62.63, 88.03, 122.49]
intGoal1 =[40.41, -90.95, -116.69, -65.00, 87.28, 109.95]
approachGoal =[144.24, -83.25, -118.34, -71.17, 87.48, 113.84]
goalPose1 =[156.33, -115.01, -114.79, -43.33, 88.00, 114.96]

##convert to radians
for it in range(len(startPose)):
    startPose_rad.append(math.radians(startPose[it]))
    approachObj_rad.append(math.radians(approachObj[it]))
    intGoal_rad.append(math.radians(intGoal[it]))
    approachGoal_rad.append(math.radians(approachGoal[it]))
    goalPose1_rad.append(math.radians(goalPose1[it]))
    
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
        for i in range(1):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=startPose_rad, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=approachObj_rad, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=intGoal_rad, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=approachGoal_rad, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=goalPose1_rad, velocities=[0]*6, time_from_start=rospy.Duration(d)))
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
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
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
