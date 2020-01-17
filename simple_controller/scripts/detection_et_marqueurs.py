#!/usr/bin/python2.7
import math, rospy
import random
import tf
import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils
from math import sqrt, degrees, cos, sin
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from matplotlib import gridspec


rospy.init_node('object_detection_and_markers', anonymous=True)
	
_goal = PoseStamped()
posxy = [0,0]
actual_angle_in_odom = 0
ObjectDetected = False
bottlePos = [[0,0],[0,0],[0,0],[0,0]]
FirstBottle = False
SecondBottle = False 
ThirdBottle = False
FourthBottle = False
cap=cv2.VideoCapture(3)
ObjectCut = False
dist = 1.1 


def node_parameter(name, default):
    value= default
    try:
        value= rospy.get_param('~' + name)
    except KeyError:
        value= default
    return value

#fonction permettant de recuperer la position du robot par rapport a odom
def get_pose_robot_in_odom(data):
    global posxy
#base_footprint est le repere du robot
    if data.transforms[0].child_frame_id == "base_footprint" and data.transforms[0].header.frame_id == "odom" : 
	    posx = data.transforms[0].transform.translation.x
	    posy = data.transforms[0].transform.translation.y
	    posxy = [posx,posy]
	    rospy.loginfo("Position robot : " + str(posx) + " " + str(posy) )

#fonction permettant de detecter les differentes bouteilles
def detection():
	    global cap
	    global ObjectDetected
	    global ObjectCut
	    global dist
	    lowH = 2
	    highH = 26
	    lowS = 160
	    highS = 255
	    lowV = 152
	    highV = 255
	    #recuperation du flux video
	    _, im =cap.read()
	    #traitement du flux video
	    blurred_frame = cv2.GaussianBlur(im, (5, 5), 0)
	    hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
	    #creation du mask dependant de la couleur orange
	    lower_orange = np.array([lowH,lowS,lowV])
	    upper_orange = np.array([highH, highS, highV])
	    mask = cv2.inRange(hsv, lower_orange, upper_orange)
	    #recuperation des contours de la bouteille sur le mask	
	    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	    cnts = imutils.grab_contours(cnts)
	    #recuperation des dimensions de l'objet repere pour obtenir ensuite le ratio (hauteur sur largeur)
	    if len(cnts)>0:
		c = max(cnts, key=cv2.contourArea)
		extLeft = tuple(c[c[:, :, 0].argmin()][0])
		extRight = tuple(c[c[:, :, 0].argmax()][0])
		extTop = tuple(c[c[:, :, 1].argmin()][0])
		extBot = tuple(c[c[:, :, 1].argmax()][0])

		ydis=sqrt((extTop[0]-extBot[0])**2+(extTop[1]-extBot[1])**2)
		xdis=sqrt((extRight[0]-extLeft[0])**2+(extRight[1]-extLeft[1])**2)
	    #calcul du ratio 
		if xdis!=0 : 
			r=ydis/xdis
		else : 
			r = 12
	  	print(" r : " + str(r))
	    #detection des bouteilles selon le ratio et les dimensions trouvees
		#detection bouteille complete
		if r< 2.2 and r>1.8 and xdis > 63 and xdis < 100 and ydis > 130 and ydis < 211  :
			print("objet complet")
			ObjectDetected = True 
			dist = 1.1
			print(ObjectDetected)
		#detection demi bouteille
		elif r<1.6 and r>1.2 and xdis > 63 and xdis < 100 and ydis > 90  and ydis < 160 : 
			print("demi objet")
			ObjectDetected = True
			ObjectCut = True 
			dist = 1.1
		#detection de la bouteille couchee
		elif r< 0.55 and r>0.45 and xdis > 130 and xdis < 211 and ydis > 63 and ydis < 100  :
			ObjectDetected = True
		#detection de la bouteille en hauteur (objet detecte de plus loin donc position du marqueur(dist) plus grande)
		elif r< 2.2 and r>1.8 and xdis > 38 and xdis < 60 and ydis > 81 and ydis < 116 :
			ObjectDetected = True
			dist = 1.8
	    cv2.imshow('thresh',mask)
	    cv2.imshow('cam',im)
	    cv2.waitKey(1)

#fonction permettant l'appel de la fonction detection() et l'appel de la fonction publication de marqueurs
def robot_orientation_in_odom(data):
    global actual_angle_in_odom
    global posxy
    global ObjectDetected
    global FirstBottle
    global SecondBottle
    global ThirdBottle
    global FourthBottle
    global ObjectCut
    num_marqueur = 0
    #appel de la fonction detection() de facon permanente
    detection()
    #posxy est la position du turtlebot dans ODOM
    quat = data.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    xa, ya, za = euler_from_quaternion(q)
    actual_angle_in_odom = za
    #appel de la fonction publication de marqueurs avec un id different
    if ObjectDetected== True :
    	position_Objet(actual_angle_in_odom, posxy)
    if FirstBottle == True and num_marqueur == 0: 
	publish_Bottle_Pos_Marker(bottlePos[0][0],bottlePos[0][1], num_marqueur, ObjectCut)
	num_marqueur = 1
    if SecondBottle == True and num_marqueur == 1 :
	publish_Bottle_Pos_Marker(bottlePos[1][0],bottlePos[1][1], num_marqueur, ObjectCut)
	num_marqueur = 2
    if ThirdBottle == True and num_marqueur == 2 : 
	publish_Bottle_Pos_Marker(bottlePos[2][0],bottlePos[2][1], num_marqueur, ObjectCut)
	num_marqueur = 3
    if FourthBottle == True and num_marqueur == 3 : 
	publish_Bottle_Pos_Marker(bottlePos[3][0],bottlePos[3][1], num_marqueur, ObjectCut)
	num_marqueur = 4

def position_Objet(angle,pos):
	global FirstBottle
        global SecondBottle
        global ThirdBottle
        global FourthBottle
	global posxy
	global ObjectDetected
	global dist
	d = 0.5
	#Position de l objet a positioner
	newpos_object = [posxy[0] + cos(actual_angle_in_odom)*dist, posxy[1] + sin(actual_angle_in_odom)*dist]
	
	if FirstBottle == False : 
		FirstBottle = True 
		bottlePos[0][0]=newpos_object[0]
		bottlePos[0][1]=newpos_object[1]
	#Si la detection de bouteille est suffisamment loin de la premiere bouteille on considere qu on en a trouve une deuxieme (cercle 50cm de rayon(d))
	if SecondBottle == False and FirstBottle == True and test_position_bouteille(0, newpos_object, bottlePos,d) :
		SecondBottle = True 
		bottlePos[1][0]=newpos_object[0]
		bottlePos[1][1]=newpos_object[1]
	#Si la detection de bouteille est suffisamment loin des deux premieres
	if ThirdBottle == False and SecondBottle == True and test_position_bouteille(1, newpos_object, bottlePos,d) : 
		ThirdBottle = True 
		bottlePos[2][0]= newpos_object[0]
		bottlePos[2][1]= newpos_object[1]
	#Si la detection de bouteille est suffisamment loin des trois premieres
	if FourthBottle == False and ThirdBottle == True and test_position_bouteille(2, newpos_object, bottlePos,d): 
		FourthBottle = True 
		bottlePos[3][0]= newpos_object[0]
		bottlePos[3][1]= newpos_object[1]
	ObjectDetected = False

#verification de la distance entre la detection de deux bouteilles afin de ne pas afficher deux fois la meme
def test_position_bouteille(Bottle_test_number, newpos_object, bottlePos,d):
	test = False
	for k in range(Bottle_test_number):
		if newpos_object[0] > bottlePos[k][0] + d or newpos_object[0] < bottlePos[k][0] - d or newpos_object[1] > bottlePos[k][1] + d or newpos_object[1] < bottlePos[k][1] - d:
			test = True
		else : 
			return(False)
	return(True)
#fonction permettant la publication du marqueur en fonction de la position du robot et de l'identifiant du marqueur
def publish_Bottle_Pos_Marker(posbouteille_x,posbouteille_y,id_num, cutted): 
    global ObjectCut
    #setup de l'objet marqueur
    rospy.loginfo(" ")
    _cmd_pub = rospy.Publisher('bottle', Marker, queue_size=10)
    marker_msg = Marker()

    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()

    marker_msg.ns = "my_point"
    marker_msg.id = id_num
    #changement de format de marqueur dans le cas d'une bouteille coupee
    if cutted : 
	marker_msg.color.r = 1.0
	marker_msg.color.g = 0.0
    	marker_msg.color.b = 0.0
	ObjectCut = False
    else : 
	marker_msg.color.r = 0.0
	marker_msg.color.g = 1.0
    	marker_msg.color.b = 0.0
    
    marker_msg.type = Marker().CUBE
    marker_msg.action = Marker().ADD
    
    #placement du marqueur a une distance d du robot dependant de la fonction detection()
    marker_msg.pose.position.x = posbouteille_x
    marker_msg.pose.position.y = posbouteille_y
    marker_msg.pose.position.z = 0.0

    #affichage marqueur RVIZ
    marker_msg.color.a = 1.0

    #definition de la taille du marqueur
    marker_msg.scale.x = 0.15
    marker_msg.scale.y = 0.15
    marker_msg.scale.z = 0.15

    marker_msg.lifetime = rospy.Duration(0)

    _cmd_pub.publish(marker_msg)



if __name__ == '__main__':	
	#Robot speed publisher
	_cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

	#Recuperation de l orientation du robot par rapport a ODOM en continu 
	rospy.Subscriber("/mobile_base/sensors/imu_data", Imu,robot_orientation_in_odom)
	#Recuperation de la position du robot par rapport a ODOM en continu 
	rospy.Subscriber("/tf", TFMessage, get_pose_robot_in_odom)
	
	# spin() simply keeps python from exiting until this node is stopped
	print("Start move.py")
	rospy.spin()


