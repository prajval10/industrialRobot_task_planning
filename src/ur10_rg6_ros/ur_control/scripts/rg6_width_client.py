#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import socket
import netifaces as ni

def measure_width():
	ni.ifaddresses('eth0')
	ip = ni.ifaddresses('eth0')[2][0]['addr']
	rospy.init_node('rg6_width_client')
	global pub , data
	pub = rospy.Publisher('/rg6_currentwidth', Float64, queue_size=1)
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_address = (ip, 50002)
	sock.bind(server_address)
	sock.listen(1)
	while True:
		connection, client_address = sock.accept()
		try:
			data = connection.recv(64)
			data = float(data)
			print "I heard : %f" % data
			pub.publish(data)
		finally:
			connection.close()

if __name__ == "__main__":
	print "Listening to RG6 server ..."
	measure_width()
	