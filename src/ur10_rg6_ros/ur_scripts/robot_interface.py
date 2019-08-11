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
from std_msgs.msg import Float32
from std_msgs.msg import Empty
import os
from ur_control.srv import RG6
from ur_control.srv import RG6_Grip
from ur_modern_driver.msg import pose
from ur_modern_driver.srv import IK

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

payload_mass = 0.2
payload_zero = 0.0
gripper_width_close = 17.0
gripper_width_open = 30.0
threshold = 0.09 # this corresponds to 5 degrees of difference in joint position
robot_file = open("/home/prajval10/catkin_ws/src/ur10_rg6_ros/ur_modern_driver/ur_scripts/robot_timing","a")

"""
## TODO after 27 July:
1. Implement feature to handle protective stop: Done-sort of
2. The OR graph is not working, check why (H1_human hyperarc)- done
3. Grasping pose is not perfect, tune this - done

## Todo after 2 Aug:
1. Check with 18 parts
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

        # Define the subscribers: Planner, Gripper and Robot IK
        self.sub_command = rospy.Subscriber("/robot_command", String, self.subRobotCommandCallback)
        self.sub_pose = rospy.Subscriber("/ObjectPose", pose, self.ObjectPoseCallback)
        self.sub_pickplace = rospy.Subscriber("/pickplace_human",String,self.pickplacehumanCallback)


    def ObjectPoseCallback(self,data):
        self.object_pose_x = data.object_pose[0]
        self.object_pose_y = data.object_pose[1]
        print "Received Object Pose x: %f, y: %f" % (self.object_pose_x, self.object_pose_y)
        pass

    def GripDetectCallback(self):
        print "Waiting for gripper detect service"
        rospy.wait_for_service('/rg6_gripper/grip_detect')
        print "Connected to gripper detect service"
        get_grip_detect = rospy.ServiceProxy('/rg6_gripper/grip_detect',RG6_Grip)
        grip_det_req = String()
        grip_det_req.data = " "
        res_value = get_grip_detect(grip_det_req) 
        self.grip_detected = res_value.gripped.data
        pass

    def pickplacehumanCallback(self,data):
        if data.data == "UnGrasp":
            print " Received Ungrasp from pickplace routine"
            self.manipulation(data.data,False)
        pass

    def subRobotCommandCallback(self, data):
        rospy.loginfo("I heard Robot command: %s", data.data)
        robCommand = data.data
        self.planner_cmd,self.agent = robCommand.split("_")
        
        # check if agent is robot
        if self.agent=="Robot":
            # TODO: check manip task or movement task - make this more generic
            if (self.planner_cmd == "Grasp" or self.planner_cmd == "UnGrasp"):
                print bcolors.BOLD+ bcolors.OKBLUE + "Manipulation task"+ bcolors.ENDC + bcolors.ENDC
                self.manipulation(self.planner_cmd, True)
            elif (self.planner_cmd == "GotoObject"):
                print bcolors.BOLD+ bcolors.OKBLUE + "Manipulation task: obj retrieval"+ bcolors.ENDC + bcolors.ENDC
                self.goToObject(self.planner_cmd) 
            else :
                print bcolors.BOLD+ bcolors.OKBLUE + "Motion task"+ bcolors.ENDC + bcolors.ENDC
                self.QueryDB(self.planner_cmd)
            pass
        pass
    pass

    def QueryDB(self,command):
        jointCommand_rad = []
        empty_list = []
        with open('/home/prajval10/catkin_ws/src/ur10_rg6_ros/ur_modern_driver/ur_scripts/db.txt') as f:
            for line in f:
                if command in line:
                    jointcommand_degree =  line.split("=")[1].rstrip()
                    new = list(jointcommand_degree.split(" "))
                    new_list = [float(i) for i in new]
                    for it in range(len(new_list)):
                        jointCommand_rad.append(math.radians(new_list[it]))   
                        pass
                    break
        self.move_to_jointcommand(jointCommand_rad,command)
        pass

    def move_to_jointcommand(self,joint_val,cmd):
        robot_file.write(str(rospy.get_rostime().secs))
        robot_file.write(" motion started \n")
        print "Sending joint command"
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        empty_traj = FollowJointTrajectoryActionGoal()
        g.trajectory.joint_names = JOINT_NAMES
        pos_flag = False
        publish_emptytraj = rospy.Publisher("/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10, latch= True)
        """
        Sending an empty trajectory message from the topic interface (not the action interface) 
        will stop the execution of all queued trajectories and enter position hold mode. 
        The stop_trajectory_duration parameter controls the duration of the stop motion. 
        """
        try:
            while not pos_flag:
                joint_states = rospy.wait_for_message("joint_states", JointState)
                joints_pos = joint_states.position
                timeDuration = 2.0
                g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
                g.trajectory.points.append(JointTrajectoryPoint(positions= joint_val, velocities=[0]*6, time_from_start=rospy.Duration(timeDuration)))   
                self.client.send_goal(g)
                self.client.wait_for_result()
                if (self.client.get_result().error_code == -100):
                    print "Robot is protective stopped."
                    self.feedback_to_planner(cmd,"false")
                    self.client.cancel_goal()
                    poll_rate = rospy.Rate(10);
                    while not rospy.is_shutdown():
                        connections = publish_emptytraj.get_num_connections()
                        if connections > 0 :    
                            rospy.loginfo("Sending empty traj : %s", empty_traj)
                            publish_emptytraj.publish(empty_traj)
                            break
                        else : 
                            print "waiting for subscribers" 
                        poll_rate.sleep();
                        pass
                    print "HUMAN: ENABLE ROBOT "
                    rospy.sleep(5.0)
                    break
                new_joint_states = rospy.wait_for_message("joint_states", JointState)
                new_joints_pos = new_joint_states.position
                pos_flag = self.position_check(list(new_joints_pos),joint_val)
                if pos_flag == True:
                    robot_file.write(str(rospy.get_rostime().secs))
                    robot_file.write(" motion ended \n")
                    self.feedback_to_planner(cmd,"true")
                    break
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        # except:
            # raise


    def manipulation(self,command,feedbck_flag):
        robot_file.write(str(rospy.get_rostime().secs))
        robot_file.write(" manipulation started \n")
        print "Waiting for gripper service"
        rospy.wait_for_service('/rg6_gripper/control_width')
        print "Connected to gripper service"
        set_gripper_width = rospy.ServiceProxy('/rg6_gripper/control_width',RG6)
        width_msg = Float64()
        payload = Float64()    
        self.grip_detected = False
        if command == "Grasp":
            width_msg.data = gripper_width_close
            payload.data = payload_mass
            set_gripper_width(width_msg,payload)
            # TODO: check for grip detect and then provide feedback
            count = 0
            while not(self.grip_detected):
                self.GripDetectCallback()
                rospy.sleep(0.1)
                count += 1
                if count == 5:
                    break
            if feedbck_flag == True:
                self.feedback_to_planner(command,"true")
        elif command == "UnGrasp":
            width_msg.data = gripper_width_open
            payload.data = payload_zero
            set_gripper_width(width_msg,payload)
            rospy.sleep(2.0)
            if feedbck_flag == True:
                self.feedback_to_planner(command,"true")

        robot_file.write(str(rospy.get_rostime().secs))
        robot_file.write(" manipulation ended \n")
        pass

    # Does the IK between origin point and pose point from camera, and goes to target point

    def goToObject(self,cmd):
        robot_file.write(str(rospy.get_rostime().secs))
        robot_file.write(" motion started \n")
        print "Go to Object pose: calling IK service"
        rospy.wait_for_service('/ur_IK_service')
        send_obj_pose = rospy.ServiceProxy('/ur_IK_service',IK)
        x_pose = Float32()
        y_pose = Float32()
        # The transformation between camera frame and robot tool frame is:
        # TODO
        x_pose.data = -self.object_pose_y
        y_pose.data = -self.object_pose_x
        send_obj_pose(x_pose,y_pose)
        # TODO: Check if robot tool pose = object pose
        rospy.sleep(3.0)
        self.feedback_to_planner(cmd,"true")
        robot_file.write(str(rospy.get_rostime().secs))
        robot_file.write(" motion ended \n")
        pass

    # Function to check if current pose - target pose < threshold
    def position_check(self,current_pose,target_pose):
        for it1,it2 in zip(current_pose,target_pose):
            if((abs(it1)-abs(it2))>threshold):
                return False
            pass
        return True
        pass

    def feedback_to_planner(self,feedback_cmd,result):
        pub = rospy.Publisher('/robot_ack', String, queue_size=10, latch= True)
        pub_command = feedback_cmd+"_Robot_"+result 
        poll_rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            if connections > 0 :    
                rospy.loginfo("Sending feedback : %s", pub_command)
                pub.publish(pub_command)
                break
            else : 
                print "waiting for subscribers" 
            poll_rate.sleep();
            pass

if __name__ == '__main__':
    try: 
        MyRobot = robot_interface()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise