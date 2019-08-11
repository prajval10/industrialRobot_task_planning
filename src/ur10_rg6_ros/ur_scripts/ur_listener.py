#!/usr/bin/env python
import rospy
import sensor_msgs.msg
from std_msgs.msg import String
pub = rospy.Publisher('/Obstacle', String, queue_size=10)
pub_command = "Stop"


def callback(data):
    dist = data.ranges
    for elem in dist:
        if elem != 3.0:
            print "Obstacle!"
            pub.publish(pub_command)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/scan", sensor_msgs.msg.LaserScan, callback)

    #spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
    rospy.spin()

