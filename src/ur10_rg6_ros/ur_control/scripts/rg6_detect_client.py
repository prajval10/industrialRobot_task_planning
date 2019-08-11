#!/usr/bin/env python

#from server_ur10.srv import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import socket
import netifaces as ni

def grip_detect():
    ni.ifaddresses('eth0')
    ip = ni.ifaddresses('eth0')[2][0]['addr']
    rospy.init_node('rg6_detect_client')
    pub = rospy.Publisher('/rg6_gripdetect', Bool, queue_size=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (ip, 50003)
    sock.bind(server_address)
    sock.listen(1)
    result = Bool()
    result.data = False        
    poll_rate = rospy.Rate(10)
    while True:
        connection, client_address = sock.accept()
        try:
            data = connection.recv(1024)
            print "I heard : %s" % data
            if data == "True":
                result.data = True   
            while not rospy.is_shutdown():
                pub_connections = pub.get_num_connections()
                if pub_connections > 0 :    
                    rospy.loginfo("Sending data : %s", data)
                    pub.publish(result)
                    break
                else : 
                    print "waiting for subscribers" 
                poll_rate.sleep();
                pass

        finally:
            connection.close()

if __name__ == "__main__":
    print "Listening to RG6 server ..."
    grip_detect()
