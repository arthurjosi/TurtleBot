#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
	Lengths = len(msg.ranges)
	#Get center laser range value 
	print(msg.ranges[Lengths/2])

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()

