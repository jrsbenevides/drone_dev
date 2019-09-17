#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import pdb
from matplotlib import pyplot as plt

def callbackOdom(data):
	
	global vector, ctOdom
	
	if data:
		
		index = ctOdom % k

		x = data.pose.pose.position.x
		y = data.pose.pose.position.y
		t = data.header.stamp.secs
		
		vector[index][0] = x
		vector[index][1] = y
		vector[index][2] = t
		
		ctOdom += 1
	
	else:
	
	   	return()


def callbackWaypoint(data2):
	
	global matWp, ctWp
	
	if data2:

		index = ctWp % k
			
		xWp = data2.pose.pose.position.x  
		yWp = data2.pose.pose.position.y
		tWp = data2.header.stamp.secs

		matWp[index][0] = xWp
		matWp[index][1] = yWp
		matWp[index][2] = tWp

		ctWp += 1

	else:

		return()


def listener():
	global vector, matWp

	vector = np.zeros((k,3))
	matWp = np.zeros((k,3))

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/bebop/transf_position',  Odometry, callbackOdom)
	rospy.Subscriber('/bebop/waypoint', Odometry, callbackWaypoint)
	rate = rospy.Rate(2) # 2hz

	while not rospy.is_shutdown():
		myPlot()
		rate.sleep()

def myPlot():

	global matWp, vector, ctOdom, ctWp

	if ctOdom > k and ctWp > k :
		plt.clf()
		plt.plot(matWp[:,0],  matWp[:,1],'bo')
		plt.plot(vector[:,0],vector[:,1],'go') 
		plt.axis([-1.2, 1.2, -1.2, 1.2])	  
		plt.draw()
		plt.pause(1e-11)

if __name__ == '__main__':

	global k, ctOdom, ctWp
	
	k = 50
	ctWp = 0
	ctOdom  = 0


	plt.ion()
	plt.show()	

	try:
		listener()
	except rospy.ROSInterruptException:
		pass
