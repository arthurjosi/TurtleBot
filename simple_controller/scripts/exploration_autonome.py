#!/usr/bin/python2.7
import math, rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('autonomous_navigation', anonymous=True)


orientation = False
orientation_ok = True

def laser_data(msg):
	global orientation
	global orientation_ok
	vel_msg = Twist()
	rand_pos_neg = 1
	v_ang = 0.4
	Lengths = len(msg.ranges)
	#Recuperation des donnees laser en face et sur les cotes 
	range_center = msg.ranges[Lengths/2]
   	range_left= msg.ranges[Lengths*2/3]
    	range_right= msg.ranges[Lengths/3]
	
	#Le robot avance tout droit en permanence sauf si un mur est devant 
	vel_msg.linear.x = 0.12
    	if range_center < 0.5 :
		vel_msg.linear.x = 0

	#Vision d un objet d'un cote donc rotation vers l autre cote (objectif ecartement des murs)
	if range_left < 0.6 and orientation == False : 
		vel_msg.angular.z = -v_ang
	elif range_right < 0.6 and orientation == False : 
		vel_msg.angular.z = v_ang
	#Vision d un objet proche en face donc rotation plus rapide vers le cote libre
	if range_center < 0.6 and orientation == False :
		if range_left >= range_right :
			vel_msg.angular.z = vel_msg.angular.z + v_ang

		elif range_left < range_right : 
			vel_msg.angular.z = vel_msg.angular.z -v_ang
	#Si trop proche d un mur le robot s arrete et va effectuer une rotation aleatoirement a gauche ou a droite jusqu a avoir le champ libre 
	if range_center < 0.5 or range_left < 0.4 or range_right < 0.4 :  
		if orientation_ok == True : 
			if random.randint(0,1) == 0 : 
				rand_pos_neg = 1
			else : 
				rand_pos_neg = -1
		orientation_ok = False
	if orientation_ok == False :
		vel_msg.angular.z = rand_pos_neg*0.25
		orientation = True
	if range_center > 1.25 and orientation and range_left > 0.6 and range_right > 0.6 :
		orientation_ok = True
		orientation = False
	# Si rien dans le champs de vision il arrete sa rotation et va repartir 
	elif range_center > 0.6 and range_left > 0.6 and range_right > 0.6 and orientation == False and orientation_ok :
		vel_msg.angular.z = 0

	# Publish continously velocity commands
	_cmd_pub.publish(vel_msg)


if __name__ == '__main__':
	print("Start move.py")	

	# Initialize command publisher:
	_cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

	# Get laser data
	rospy.Subscriber('/scan', LaserScan, laser_data)
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()



