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
from std_msgs.msg import Float64
import os
from ur_control.srv import RG6


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

payload_mass = 0.2
payload_zero = 0.0
gripper_width_close = 17.0
gripper_width_open = 27.0

"""
## TODO:
# 1. Implement emergency stop
# 2. Make commands more generic
# 3. Clean the code

3 July changes:
Not using grip detect boolean variable

"""
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class robot_interface(object):    
    
    def __init__(self):
        rospy.init_node("robot_interface", anonymous=True, disable_signals=True)
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        self.sub = rospy.Subscriber("/robot_command", String, self.subRobotCommandCallback)
        self.sub1 = rospy.Subscriber("/rg6_gripdetect", String, self.GripCallback)

    def GripCallback(self,data):
        global grip_detect
        grip_detect = False
        if data.data == "True":
            grip_detect = True
        pass

    def subRobotCommandCallback(self, data):
        rospy.loginfo("I heard Robot command: %s", data.data)
        global cmd
        robCommand = data.data
        cmd,agent = robCommand.split("_")
        
        # check if agent is robot
        if agent=="Robot":
            # TODO: check manip task or movement task - make this more generic
            if (cmd == "Grasp" or cmd == "UnGrasp"):
                print bcolors.BOLD+ bcolors.OKBLUE + "Manipulation task"+ bcolors.ENDC + bcolors.ENDC
                self.manipulation(cmd)
            if (cmd == "Goto-object"):
                print bcolors.BOLD+ bcolors.OKBLUE + "Manipulation task: obj retrieval"+ bcolors.ENDC + bcolors.ENDC
                self.goToObject();
            else:
                print bcolors.BOLD+ bcolors.OKBLUE + "Motion task"+ bcolors.ENDC + bcolors.ENDC
                self.QueryDB(cmd) 
            pass
        pass
    pass

    def QueryDB(self,command):
        self.jointCommand_rad = []
        empty_list = []
        with open('/home/prajval10/catkin_ws/src/ur10_rg6_ros/ur_modern_driver/ur_scripts/db.txt') as f:
            for line in f:
                if command in line:
                    jointcommand_degree =  line.split("=")[1].rstrip()

        new = list(jointcommand_degree.split(" "))
        new_list = [float(i) for i in new]
        for it in range(len(new_list)):
            self.jointCommand_rad.append(math.radians(new_list[it]))   
            pass

        self.move_to_jointcommand(self.jointCommand_rad)
        pass

    def move_to_jointcommand(self,joint_val):
        
        print "in joint command"
        self.g = FollowJointTrajectoryGoal()
        self.g.trajectory = JointTrajectory()
        self.g.trajectory.joint_names = JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            timeDuration = 2.0
            self.g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            self.g.trajectory.points.append(JointTrajectoryPoint(positions= joint_val, velocities=[0]*6, time_from_start=rospy.Duration(timeDuration)))   
            self.client.send_goal(self.g)
            self.client.wait_for_result()
            self.feedback_to_planner()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise


    def manipulation(self,command):
        global grip_detect
        print "Calling gripper service"
        rospy.wait_for_service('/rg6_gripper/control_width')
        set_gripper_width = rospy.ServiceProxy('/rg6_gripper/control_width',RG6)
        width_msg = Float64()
        payload = Float64()    
        if command == "Grasp":
            width_msg.data = gripper_width_close
            payload.data = payload_mass
            set_gripper_width(width_msg,payload)
            rospy.sleep(2.0)
            # if grip_detect:
            self.feedback_to_planner()
        elif command == "UnGrasp":
            width_msg.data = gripper_width_open
            payload.data = payload_zero
            set_gripper_width(width_msg,payload)
            rospy.sleep(2.0)
            # if grip_detect==False:
            self.feedback_to_planner()

        pass

    def goToObject():
        print "Go to Object pose:"
        
        pass

    def feedback_to_planner(self):
        global cmd
        self.pub = rospy.Publisher('/robot_ack', String, queue_size=10, latch= True)
        self.pub_command = cmd+"_Robot_true" 
        self.poll_rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            self.connections = self.pub.get_num_connections()
            if self.connections > 0 :    
                rospy.loginfo("Sending feedback : %s", self.pub_command)
                self.pub.publish(self.pub_command)
                break
            else : 
                print "waiting for subscribers" 
            self.poll_rate.sleep();
            pass

def main():
    global payload_mass, gripper_width
    global grip_detect
    
    try: 
        MyRobot = robot_interface()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
   main()