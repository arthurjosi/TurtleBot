#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


def publishOnce():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('marker_pub', anonymous=True)
    rate = rospy.Rate(10)
    _cmd_pub = rospy.Publisher('bottle', Marker, queue_size=10)
    marker_msg = [0,0,0,0]
    for k in range(3) : 
	    marker_msg[k] = Marker()
	    marker_msg[k].header.frame_id = "map"
	    marker_msg[k].header.stamp = rospy.Time.now()

	    marker_msg[k].ns = "my_point"
	    marker_msg[k].id = 1
	    #marker_msg.type = Marker().MESH_RESOURCE
	    marker_msg[k].type = Marker().CUBE
	    marker_msg[k].action = Marker().ADD
	    #marker_msg.mesh_resource = "/home/bot/catkin_ws/src/using_markers/src/Bottle.dae"
	
	    marker_msg[k].pose.position.x = k
	    marker_msg[k].pose.position.y = k
	    marker_msg[k].pose.position.z = k

	    marker_msg[k].color.r = 0.0
	    marker_msg[k].color.g = 1.0
	    marker_msg[k].color.b = 0.0
	    marker_msg[k].color.a = 1.0

	    marker_msg[k].scale.x = 0.1
	    marker_msg[k].scale.y = 0.1
	    marker_msg[k].scale.z = 0.1
	    for i in range(5) :
		rate.sleep()

	    _cmd_pub.publish(marker_msg[k])
	    rate.sleep()
	
    marker_msg = Marker()
    other = Marker()
   # rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, callback)

    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()

    marker_msg.ns = "my_point"
    marker_msg.id = 1
    #marker_msg.type = Marker().MESH_RESOURCE
    marker_msg.type = Marker().CUBE
    marker_msg.action = Marker().ADD
    #marker_msg.mesh_resource = "/home/bot/catkin_ws/src/using_markers/src/Bottle.dae"
	
    marker_msg.pose.position.x = 1.0
    marker_msg.pose.position.y = 1.0
    marker_msg.pose.position.z = 1.0

    marker_msg.color.r = 0.0
    marker_msg.color.g = 1.0
    marker_msg.color.b = 0.0
    marker_msg.color.a = 1.0

    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    

    marker_msg.lifetime = rospy.Duration(0)
    other.lifetime = rospy.Duration(0)
    rospy.loginfo("_cmd_pub -> " +  str(_cmd_pub))


    for i in range(5) :
	rate.sleep()

    _cmd_pub.publish(marker_msg[k])
    rate.sleep()
    _cmd_pub.publish(other)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    publishOnce()
