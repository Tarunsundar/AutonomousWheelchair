#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Tarunsundar
from opencv_apps.msg import MomentArrayStamped
import random
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan

import tf2_ros
import rospy
import actionlib


#example based on cv_bridge tutorials
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

flag =0
flaginfo =""
edge1 = 0
edge2 = 0
doorWayDetected = False
angularAlignment = False
moveTowardsDoorway = False
crossedDoorWay = False
diffE1 = 0.0 
diffE2 = 0.0
maxRangeVal = 0
midRangeVal = 0
stackEdge = []
stackDiff = []
	
#callback method for publishing angular alignment messages of the robot
def angularAdjustment(msg_twist, pub): 
	global flaginfo 
	global edge1
	global edge2
	current_angle = 0
	global moveTowardsDoorway
	global flaginfo
	adjustment = (edge1/4 + edge2/4)/2
	global maxRangeVal
	global midRangeVal
	global angularAlignment
	speed = 5.8
	
	angular_speed = speed*2*3.14/360	
	clockwise = False # boolean is either True or false
	
	#We wont use linear components
	msg_twist.linear.x=0
	msg_twist.linear.y=0
	msg_twist.linear.z=0
	msg_twist.angular.x = 0
	msg_twist.angular.y = 0
	print("edge1:", edge1, "edge2:", edge2)
	print("adjustment angle =", adjustment)
	# Checking if our movement is CW or CCW
	if adjustment  < midRangeVal/4-0.75:
		clockwise = True
	else:
		clockwise = False
	if int(adjustment) != midRangeVal/4: #midRange value by 4 is equal to 90 for the current robo configuration 
		flaginfo = "doorway found, turning to face doorway"
		if clockwise:
			msg_twist.angular.z = -angular_speed
			print("turning clockwise")
		else:
			msg_twist.angular.z = angular_speed
			print("turning anti clockwise")
			pub.publish(msg_twist)
	else:
		print("Robot alignement with the doorway complete!!")
		moveTowardsDoorway = True
		angularAlignment = False
		moveTowardsDoor(msg_twist)


#method if doorway not found
def findDoorway():
	# Initialise the ROS system and become a node
	rospy.init_node('publish_velocity')
	
	# Create as publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
	
	msg_twist = Twist()
	global edge2
	if(edge2 == 0):
		msg_twist.angular.z = 0.5
	else:
		msg_twist.angular.z = 0.0

#method for robot find the doorway
def moveTowardsDoor(msg_twist):
	global diffE1
	global diffE2
	global edge1
	global edge2
	global moveTowardsDoorway
	global flaginfo
	flaginfo = "Moving Towards the doorway"
	print("edge1= ", edge1, "diffE1= ", diffE1,"edge2= ", edge2, "diffE2= ", diffE2)
	msg_twist.linear.x = 6.9
			
#method for the robot's end condition
def endCondition(msg_twist, pub):
	import time
	global flaginfo
	flaginfo = "Crossed the doorway!!"
	msg_twist.linear.x = 0
	msg_twist.angular.z = 0
	pub.publish(msg_twist) #publishing velocity message to robot here as the robot would break out of main loop after this method
	
#method to stop the robot's movements when terminated
def terminate(msg_twist, pub):
	global flaginfo 
	flaginfo = "program terminating!!"
	msg_twist.linear.x = 0
	msg_twist.angular.z = 0
	pub.publish(msg_twist) #publishing velocity message to robot here as the robot would break out of main loop after this method 
	

#callback method to check the laserscanner values
def laser_cb(msg):
	global doorWayDetected 
	global moveTowardsDoorway
	global crossedDoorWay
	global edge1
	global edge2
	i = 0
	global diffE1 
	global diffE2
	global midRangeVal
	global maxRangeVal
	global angularAlignment
	maxRangeVal = len(msg.ranges)
	midRangeVal = maxRangeVal/2
	scannedAll = False 
	global stackEdge 
	global stackDiff
	frontmidVal = 0.0
	adjustment = (edge1/4 + edge2/4)/2
	if crossedDoorWay == False:
		frontmidVal = msg.ranges[359]
		while i <= maxRangeVal:
			if(i == 718):
				scannedAll = True
				break
			else :
				i+=1
			diff = abs(msg.ranges[i]-msg.ranges[i+1])
			if(diff > 2): 
				stackEdge.append(i) 
				stackDiff.append(diff)
				print("edge: ",i)
			if(len(stackEdge) ==2):
				edge1 = stackEdge.pop()
				diffE1 = stackDiff.pop()
				edge2 = stackEdge.pop()
				diffE2 = stackDiff.pop()
				doorWayDetected = True
	if doorWayDetected and adjustment < midRangeVal/4 -0.75  or adjustment > midRangeVal/4:
		moveTowardsDoorway = False
		angularAlignment = True
	else:
		moveTowardsDoorway = True
		angularAlignment = False
	if moveTowardsDoorway == True and frontmidVal<1.5:
		print("diff E1", diffE1, "diffE2", diffE2)
		moveTowardsDoorway = False
		crossedDoorWay = True

#this is the main method that subscribes and publishes to topics of the robot in order to find, move and cross doorway
def crossDoorWay():
	global flag	
	global flaginfo
	global doorWayDetected
	global moveTowardsDoorway
	global crossedDoorWay
	global midRangeVal
	global angularAlignment
	# Initialise the ROS system and become a node
	rospy.init_node('publish_velocity')
	
	# Create as publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

	# Create a subscriber objecto
	rospy.Subscriber('/scan', LaserScan, laser_cb)
	
	# Loop at 2Hz until the node is shutdown
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Create and fill in the message.  
		msg_twist = Twist()
		
		#setting up the finite state machine 
		if(doorWayDetected == True and angularAlignment == True and not crossedDoorWay): # if doorway found set flag accordingly
			flag = 1
		elif(moveTowardsDoorway == True and not crossedDoorWay): #move towards doorway
			flag = 2
		elif(crossedDoorWay == True): #this condition checks if crossed the doorway
			flag = 3
		else:
			flag = 0 
		
		#code to implement the finite state machine
		if(flag == 0): #if no object found keep turning
			flaginfo = "looking for the doorway"
			print("doorway detected= ", doorWayDetected," alignment = " ,angularAlignment)
		elif(flag == 1): #if doorway found
			angularAdjustment(msg_twist, pub)
		elif(flag == 2):
			moveTowardsDoor(msg_twist)
		elif(flag == 3):
			endCondition(msg_twist, pub)
			print("flaginfo =",flaginfo)
			break
		else:
			rospy.logerr("something went wrong nothing detected for flag")#need to check what's causing the issue here.
		
		#publish the message
		pub.publish(msg_twist)
		
		#print flaginfo
		print("flaginfo =",flaginfo)
		
		# Send a message to rosout with the details
		rospy.loginfo("Current stauts of the robot = %s", flaginfo)
		
		# Wait until it's time for another iteration
		rate.sleep()
	terminate(msg_twist, pub)
	print("flaginfo =",flaginfo)


if __name__ == '__main__':
	try:
		crossDoorWay()
	except rospy.ROSInterruptException:
		pass
