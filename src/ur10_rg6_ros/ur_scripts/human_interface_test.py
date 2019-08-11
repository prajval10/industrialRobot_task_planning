#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sensor_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float64
import operator
import socket
import netifaces as ni
import time
from std_msgs.msg import Float64MultiArray
from ur_modern_driver.msg import pose
import threading
import sys
try: import tkinter
except ImportError:
    import Tkinter as tkinter
    import ttk
    import Queue as queue
else:
    from tkinter import ttk
    import queue

trigger_cam_on = "set_standard_digital_out(0, True)"
trigger_cam_off = "set_standard_digital_out(0, False)"
camera_host = "192.168.1.9"
camera_port = 50003
ux_flag = False
ux_event = threading.Event() # global event for control transfer between threads
ux_event.clear()
vision_file = open("/home/prajval10/catkin_ws/src/ur10_rg6_ros/ur_modern_driver/ur_scripts/vision_timing","a")

class HumanInterface(object):
    """docstring for HumanInterface"""
    def __init__(self):
        super(HumanInterface, self).__init__()
        rospy.init_node("human_interface", anonymous=True, disable_signals=True)
        self.sub = rospy.Subscriber("/human_command", String, self.subHumanCommandCallback)
        
    
    def subHumanCommandCallback(self,data):
        rospy.loginfo("I heard : %s", data.data)
        cmd,agent = data.data.split("_")
        if (cmd=="Handover"):
            que.put('handover')
            self.callCameraService(cmd)
        elif (cmd == "PickPlace"):
            self.performPickPlace(cmd)
        pass

    def callCameraService(self,cmd_data):
        self.pub = rospy.Publisher("/ur_driver/URScript",String, queue_size=10)
        vision_file.write(str(rospy.get_rostime().secs))
        vision_file.write(" vision started \n")

        print "Waiting for object pose from camera"
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0) #if camera doesnt respond in 1 seconds, retry
        sock.connect((camera_host,camera_port))
        print "connected to camera"

        poll_rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            self.connections = self.pub.get_num_connections()
            if self.connections > 0 :    
                flag = False
                while not flag:
                    rospy.loginfo("Triggering camera:")
                    self.pub.publish(trigger_cam_off)
                    time.sleep(0.1)
                    self.pub.publish(trigger_cam_on) #the camera only responds to a rising edge
                    print "Waiting to receive data"
                    
                    try:
                        data = sock.recv(1024)
                        pass
                    except socket.timeout as e:

                        err = e.args[0]
                        if err == 'timed out':
                            time.sleep(1)
                            print "Socket timed out, try again"
                            continue
                        else:
                            print e
                            sock.close()
                            sys.exit(1)
                    else:        
                        if len(data) == 0:
                            print "Error, close server"
                            sock.close()
                            sys.exit(1)
                        else:
                            print "Received Pose successfully"
                            que.put('handover done')
                            vision_file.write(str(rospy.get_rostime().secs))
                            vision_file.write(" vision ended \n")
            
                            flag = True
                            x = data.split(",")
                            x_pose = x[0]
                            y_pose = x[1]
                            print x_pose, y_pose
                            sock.close()
                            self.feedbackToPlanner(cmd_data)
                            self.SendPoseCommands(x_pose,y_pose)
                    pass
                print "done"
                break
            else : 
                print "waiting for subscribers" 
            poll_rate.sleep();
            pass
        pass

    def performPickPlace(self,cmd):
        # global ux_flag
        # ux_flag = False
        ux_event.clear()
        print "Human perform pick and place"
        print "Please collect part from robot"
        que.put("enable_robot")
        # while not ux_flag:
        #     pass
        # ux_flag = False
        ux_event.wait()
        ux_event.clear()
        que.put("collect part")
        ux_event.wait()
        ux_event.clear()
        rospy.sleep(2.0)
        # raw_input("Press Enter to continue...")
        pub = rospy.Publisher("/pickplace_human",String, queue_size=10, latch= True)
        pub_str = String()
        pub_str.data = "UnGrasp"
        poll_rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            if connections > 0 :    
                rospy.loginfo("Sending UnGrasp")
                pub.publish(pub_str)
                break
            else : 
                print "waiting for subscribers" 
            poll_rate.sleep();
            pass

        rospy.sleep(2.0)
        print "Please come outside the robot workspace"
        que.put("safe_dist")
        # while not ux_flag:
        #     pass
        # ux_flag = False
        ux_event.wait()
        ux_event.clear()
        # raw_input("Press Enter to continue...")
        self.feedbackToPlanner(cmd)
        pass

        
    def SendPoseCommands(self,x,y):
        self.pose_pub = rospy.Publisher('/ObjectPose', pose , queue_size=10, latch= True)
        pub_cmd = [float(x), float(y)]
        ObjPose = pose()
        ObjPose.object_pose = pub_cmd
        poll_rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            self.connections = self.pose_pub.get_num_connections()
            if self.connections > 0 :    
                rospy.loginfo("Sending pose")
                self.pose_pub.publish(ObjPose)
                break
            else : 
                print "waiting for subscribers" 
            poll_rate.sleep();
            pass
        pass    

    def feedbackToPlanner(self, data):
        self.pub = rospy.Publisher('/HRecAction', String, queue_size=10, latch= True)
        self.pub_command = data 
        poll_rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            self.connections = self.pub.get_num_connections()
            if self.connections > 0 :    
                rospy.loginfo("Sending feedback : %s", self.pub_command)
                self.pub.publish(self.pub_command)
                break
            else : 
                print "waiting for subscribers" 
            poll_rate.sleep();
            pass



class GUI_Core(object):

    def __init__(self):
        self.root = tkinter.Tk()
        self.root.title("User Interface")
        self.root.attributes("-topmost", True)
        self.root.iconbitmap('@/home/prajval10/gui-robot/icon_goal.xbm')
        self.frame = ttk.Frame(self.root, height = 700, width = 850)
        self.frame.pack()

        self.slovak = False
        
        # Insert Text 
        self.label = ttk.Label(self.frame, text='Please press START to begin task')
        self.label.config(font=("Helvetica", 35))
        self.label.config(wraplength = 600)
        self.label.place(x=10,y= 300)
        
        # Insert Buttons
        self.b_start = tkinter.Button(self.frame, text='Start',width='20')
        self.b_start.config(font=("Helvetica", 15))
        self.b_start['command'] = self.start_thread
        self.b_start.place(x=200, y= 500)

        self.b_ok = tkinter.Button(self.frame, text='OK',width='20')
        self.b_ok.config(font=("Helvetica", 15))
        self.b_ok['command'] = self.set_flag_
        self.b_ok.place(x=450, y= 500)
        self.b_ok['state'] = 'disable'        

        self.photo = tkinter.PhotoImage(file="/home/prajval10/gui-robot/slovak.png")
        self.button_lang = tkinter.Button(self.frame, image = self.photo, command = self.lang_change_eng_slo)
        self.button_lang.place(x= 700, y = 150)

        self.photo2 = tkinter.PhotoImage(file="/home/prajval10/gui-robot/england.png")
        self.button_lang1 = tkinter.Button(self.frame, image = self.photo2, command = self.lang_change_slo_eng)
        self.button_lang1.place(x= 770, y = 150)

        # Insert Logos
        self.logo = tkinter.PhotoImage(file="/home/prajval10/gui-robot/logo.png")
        self.w1 = tkinter.Label(self.frame, image=self.logo)
        self.w1.place(x=0, y=0)

        self.logo_unige = tkinter.PhotoImage(file="/home/prajval10/gui-robot/unige_logo.png")
        self.w2 = tkinter.Label(self.frame, image=self.logo_unige)
        self.w2.place(x=600, y=0)

    def set_flag_(self):
        # global ux_flag
        # ux_flag = True
        ux_event.set()
        pass

    def lang_change_eng_slo(self):
        self.slovak = True
        self.b_start.config(text="Štart")
        self.label.config(text="Stlačte tlačidlo Štart na spustenie úlohy")
        pass

    def lang_change_slo_eng(self):
        self.slovak = False
        self.b_start.config(text="Start")
        self.label.config(text="Please press START to begin task")
        pass

    
    def start_thread(self):
        self.b_start['state'] = 'disable'
        # self.label['text'] = 'Please check the part and place it below the camera'
        # create then start a secondary thread to run arbitrary()
        self.secondary_thread = threading.Thread(target=ros_part)
        self.secondary_thread.start()
        # check the Queue in 10ms
        self.frame.after(10, self.check_que)

    def stop(self):
        self.secondary_thread.join()
    
    def check_que(self):
        while True:
            try: x = que.get_nowait()
            except queue.Empty:
                self.frame.after(20, self.check_que)
                break
            else: # continue from the try suite
                if self.slovak == True:
                    if x == "handover":
                        self.b_ok['state'] = 'disable'
                        self.label.config(text='Pozrite sa a umiestnite časť pod fotoaparát')        

                    elif x == "handover done":
                        self.label.config(text='Úloha odovzdania prebehla úspešne')
                    
                    elif x == "enable_robot":
                        self.b_ok['state'] = 'normal'
                        self.label.config(text = "Robot je ochranný zastavený. Prosím povoľte robot")

                    elif x == "collect part":
                        self.label.config(text = "Prosím zhromažďujte Par")
                        
                    elif x == "safe_dist":
                        self.label.config(text = "Presuňte sa z pracovného priestoru robota")
                        
                elif self.slovak == False:
                    if x == "handover":
                        self.b_ok['state'] = 'disable'
                        self.label.config(text='Please inspect and place part below camera')        
                    elif x == "handover done":
                        
                        self.label.config(text='Handover task done successfully')
                    elif x == "enable_robot":
                        self.b_ok['state'] = 'normal'
                        self.label.config(text = "Robot is protective stopped. Please Enable Robot")
                        
                    elif x == "collect part":
                        self.label.config(text = "Please Collect Part")
                        
                    elif x == "safe_dist":
                        self.label.config(text = "Please move out of robot workspace")
                
                elif x == "close":
                    self.b_start['state'] = 'normal'
                    break


def ros_part():
    try: 
        print "Human interface open, waiting for command from planner"
        human_agent = HumanInterface()
        # rospy.spin() This doesnt work when Ctrl+C is pressed 
    except:
        pass
    pass

if __name__ == '__main__':
    que = queue.Queue()
    gui = GUI_Core() # see GUI_Core's __init__ method
    try:
        gui.root.mainloop()
        pass
    except KeyboardInterrupt:
        gui.stop()
        rospy.signal_shutdown("KeyboardInterrupt")
        gui.root.destroy()
        raise 
    
    