/*
 * droneSystem.h
 *
 *  Created on: 16/06/2017
 *      Author: roberto
 */

#ifndef DRONESYSTEM_H_
#define DRONESYSTEM_H_


#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>

#include "drone/drone.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"

#include "Eigen/Dense"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>


namespace DRONE {

class System {

	ros::NodeHandle n;

	ros::Publisher cmd_vel_publisher;
	ros::Publisher transfPosition_publisher;

	ros::Subscriber odom_subscriber;
	ros::Subscriber waypoint_subscriber;
	ros::Subscriber fix_subscriber;
	ros::Subscriber joy_subscriber;
	ros::Subscriber vicon_subscriber;

	private:
	  
	double wAng;

	public:
		
	System();
	~System ();

	void control();
		
	long int count;
	long int countEKF;
		
	bool 	 flagEnable;
	bool 	 flagControllerStarted;
	bool 	 flagTwist;
		
	double 	 PI;
	double 	 vxAmpl;
	double 	 vyAmpl;
	double 	 vzAmpl;
	double 	 angAmpl;
	double 	 f;
	double 	 amplitude;
	double 	 velMed;
		
	string	 trajectory;
	string   controlSelect;
	string   sensorSelect;

	Drone    drone;


	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
	void waypointCallback(const nav_msgs::Odometry::ConstPtr& waypoint); //alterado
	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& vicon);
	void globalToLocalPosition(const Vector3axes& positionValue, const VectorQuat& orientationValue, const Vector3axes& linearVelValue,const Vector3axes& angularVelValue);
	void initDroneSystemParam(void);
	void loadTopics(ros::NodeHandle &n);
	void loadSettings(ros::NodeHandle &n);
	void setTrajectory(const string& trajectoryInput);
	void setControlSelect(const string& controlSelectInput);
	void setSensorSelect(const string& sensorSelectInput);
	void setAmplitude(const double& amplitudeValue);
	void setVelMed(const double& velMedValue);
	void bootVicon(const double& timeValue);
}; /*class System */
} /*namespace DRONE*/


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
