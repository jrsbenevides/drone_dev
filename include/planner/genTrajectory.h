/*
 * droneSystem.h
 *
 *  Created on: 16/06/2017
 *      Author: roberto
 */

#ifndef GENTRAJECTORY_H_
#define GENTRAJECTORY_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include "angles/angles.h"
#include "drone/operations.h"
#include "drone/definitions.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include "drone/drone.h"
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>

// #include "drone/drone.h"

using namespace std;

namespace DRONE {

	class Planner {

	    ros::NodeHandle n;
	    ros::Publisher  waypoint_publisher;
	    ros::Subscriber joy_subscriber;

	  private:

	  	bool   isControlStarted;
		double PI;
		double t;
		double wAng; // w = 2*pi*vel_media/(6.097*a);
		double startTime;
		bool   firstTimePass;
	  
	  public:
		
		double 		amplitude;
		double 		velMed;
		string 		trajectory;
		VectorFive 	poseDesired;
		Vector3axes cTx, cTy,cTz, cTyaw;

		Planner();
		~Planner ();
		
		// Drone drone;

		void setIsControlStarted(bool state);
		void setposeDesired(VectorFive poseDesiredValue);
		void setTrajectoryCoefficients(void);
		bool getIsControlStarted(void);
		VectorFive getposeDesired(void);
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void loadTopics(ros::NodeHandle &n);
		void loadSettings(ros::NodeHandle &n);
		void refreshWang(void);
		void TrajPlanner(void);
		void setTrajectory(const string& trajectoryInput);
		void angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll);
	};


} // namespace DRONE


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
