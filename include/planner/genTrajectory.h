/*
 * droneSystem.h
 *
 *  Created on: 16/09/2019
 *      Author: jrsbenevides
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
		double wAng;
		double startTime;
		bool   isFirstTimePass;
	  
	  public:
		
		double 		amplitude;
		double 		velMed;
		string 		trajectory;
		VectorFive 	poseDesired;
		Vector3axes cTx, cTy,cTz, cTyaw;

		Planner();
		~Planner ();
		
		// Drone drone;
		void initPlanner(void);
		void setTrajectory(const string& trajectoryInput);
		void setIsControlStarted(bool state);
		void setIsFirstTimePass(bool state);
		void setposeDesired(VectorFive poseDesiredValue);
		void setTrajectoryCoefficients(void);
		bool getIsControlStarted(void);
		bool getIsFirstTimePass(void);
		VectorFive getposeDesired(void);
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void loadTopics(ros::NodeHandle &n);
		void loadSettings(ros::NodeHandle &n);
		void refreshWang(void);
		void TrajPlanner(void);
		void angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll);
	};


} // namespace DRONE


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
