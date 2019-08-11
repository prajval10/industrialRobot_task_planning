#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import socket
import netifaces as ni
import time

def send_pose():
	ni.ifaddresses('eth0')
	ip = ni.ifaddresses('eth0')[2][0]['addr']
	rospy.init_node('object_pose_client')
	global pub , data
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_address = (ip, 50003)
	sock.bind(server_address)
	sock.listen(1)
	x = 110
	y = 100
	while True:
		connection, client_address = sock.accept()
		try:
			data = connection.recv(1024)
			print data
			time.sleep(1)
			if data == "Asking_pose":
				connection.send("(%d,%d)" % (x,y))
			
		finally:
			connection.close()

if __name__ == "__main__":
	send_pose()
	