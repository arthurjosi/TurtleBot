#!/usr/bin/python2.7
import math, rospy
import tf
from math import atan2, sqrt, degrees
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan


# Initialize ROS::node
rospy.init_node('move', anonymous=True)

# Initialize global variable
_trans = tf.TransformListener()
_goal = PoseStamped()
my_goal=[5,5,False]
my_goal_in_odom =[5,5]
posxy = [0,0]
orientation_done = False
rotation = True
test_x_negative = True
rotation_needed = 0

# Initialize node parrameters (parrameter name, default value)
def node_parameter(name, default):
    value= default
    try:
        value= rospy.get_param('~' + name)
    except KeyError:
        value= default
    return value

_goal_topic= node_parameter('goal_topic', '/move_base_simple/goal')
_goal_frame_id= node_parameter('goal_frame_id', 'odom')
_cmd_topic= node_parameter('cmd_topic', '/cmd_vel')
_cmd_frame_id= node_parameter('cmd_frame_id', 'base_footprint')
# Initialize command publisher:
_cmd_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)


def log_goal(data):
    global my_goal
    global my_goal_in_odom
    rospy.loginfo( 'Goal ' + data.header.frame_id +": (" + str(data.pose.position.x) +", "+str(data.pose.position.y) +", "+str(data.pose.position.z) +")" )
    if data.header.frame_id == "base_footprint" :
	#Nouvel objectif :
	if data.pose.position.x != my_goal[0] :
		rotation = True
		my_goal[2] = True
	my_goal[0] = data.pose.position.x
	my_goal[1] = data.pose.position.y
    if data.header.frame_id == "odom" :
	my_goal_in_odom = [data.pose.position.x,data.pose.position.y]
	
	
# Subscribe to topic to get a goal position with goal_subscriber function
def goal_subscriber(data):
    global _goal
    log_goal(data)
    _goal= _trans.transformPose(_goal_frame_id, data)
    log_goal(_goal)

def get_pose_frame_in_odom(data):
    global posxy
    if data.transforms[0].child_frame_id == "base_footprint" and data.transforms[0].header.frame_id == "odom" : 
	    posx = data.transforms[0].transform.translation.x
	    posy = data.transforms[0].transform.translation.y
	    posxy = [posx,posy]
	    #rospy.loginfo("Position : " + str(posx) + " " + str(posy) )

def orientation(aim_angle_in_base_footprint, actual_angle_in_odom,vel_msg):
	global rotation 
	global rotation_needed
	global my_goal
	global test_x_negative
	global orientation_done
	if rotation == True :
		w = actual_angle_in_odom + aim_angle_in_base_footprint
		if w > 180 : 
			rotation_needed = - w%180 - 180 
		elif w < -180 :
			rotation_needed = + w%180 + 180
		else : 
			rotation_needed = w
		rotation = False
        #rospy.loginfo("Rotation needed : " + str(rotation_needed))
	if my_goal[0] < 0 and test_x_negative == True :
		test_x_negative = False
		if rotation_needed > 0 :
			rotation_needed = rotation_needed - 180
		else : 
			rotation_needed = rotation_needed + 180
		
	if rotation_needed >= 0 : 
		if actual_angle_in_odom < rotation_needed and actual_angle_in_odom > rotation_needed - 180 :
			vel_msg.angular.z = 0.3
		else :
			vel_msg.angular.z = -0.3
	elif rotation_needed < 0 : 
		if actual_angle_in_odom  < rotation_needed + 180 and actual_angle_in_odom > rotation_needed :
			vel_msg.angular.z = -0.3
		else : 
			vel_msg.angular.z = 0.3
	if actual_angle_in_odom < rotation_needed + 5 and actual_angle_in_odom > rotation_needed - 5 :
		
		orientation_done = True

def move_forward(vel_msg):
	global my_goal
	global posxy
	global my_goal_in_odom

   	a2 =  abs(my_goal_in_odom[0]-posxy[0])
   	b2 = abs(my_goal_in_odom[1]-posxy[1])
	
	aim_dist = sqrt(a2**2 + b2**2)
	rospy.loginfo("actual_distance :" + str(aim_dist))
	vel_msg.linear.x = 0.3
	if aim_dist < 0.3 : 
		my_goal[2] = False 
		orientation_done = False

def laser_data(msg):
	global orientation_done
	Lengths = len(msg.ranges)
	#Get center laser range value 
	print(msg.ranges[Lengths/2])
	#if orientation_done == True : 
		
	

def move_com(data):
    global increase
    global aim_angle
    global my_goal
    global orientation_done
    vel_msg = Twist()
    quat = data.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    xa, ya, za = euler_from_quaternion(q)
    actual_angle_in_odom = degrees(za)
    #rospy.loginfo("actual_angle_in_odom : " + str(actual_angle_in_odom))
    if my_goal[2] : 
    #Angle de rotation qu il faut dans base_footprint pour aller a l objectif
            if my_goal[1] < 0 and my_goal[0] < 0: 
	    	aim_angle_in_base_footprint = degrees(atan2(my_goal[1],my_goal[0])) + 180
	    elif my_goal[1] > 0 and my_goal[0] < 0 :
		aim_angle_in_base_footprint = degrees(atan2(my_goal[1],my_goal[0])) - 180
	    else : 
		aim_angle_in_base_footprint = degrees(atan2(my_goal[1],my_goal[0]))
	    #rospy.loginfo("aim_angle_in_base_footprint : " + str(aim_angle_in_base_footprint))

	    #rospy.loginfo("actual_angle_in_odom :" + str(actual_angle_in_odom))
	    orientation (aim_angle_in_base_footprint, actual_angle_in_odom,vel_msg)
	    if orientation_done == True:
	    	move_forward(vel_msg)
	    
	    _cmd_pub.publish(vel_msg)
	

rospy.Subscriber(_goal_topic, PoseStamped, goal_subscriber)

#rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, seccallback)

rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, move_com)

# Publish continously velocity commands
rospy.Subscriber("/tf", TFMessage, get_pose_frame_in_odom)

# Get laser data
rospy.Subscriber('/scan', LaserScan, laser_data)

def move_command(data):
    log_goal(_goal)
    if _goal.header.frame_id != '' :
        local_goal= _trans.transformPose(_cmd_frame_id, _goal)
    else :
        local_goal= _goal
    log_goal(local_goal)


    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)

rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )



# spin() simply keeps python from exiting until this node is stopped
print("Start move.py")
