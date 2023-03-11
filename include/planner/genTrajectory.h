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
#include "planner/trajectoryTypes.hpp"
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>

#define PRINT_LOG 0

namespace DRONE {

class Planner {

	public:

	ros::NodeHandle n;
	ros::Publisher  waypoint_publisher;
	ros::Subscriber joy_subscriber;
		
	Planner();
	~Planner ();
		
	void init(void);
	void setIsControlStarted(bool state);
	bool getIsControlStarted(void);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void loadTopics(ros::NodeHandle &n);
	void loadSettings(ros::NodeHandle &n);
	void TrajPlanner(void);
	void angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll);

private:

	void setposeDesired(VectorFive poseDesiredValue);

	void planEightShape(nav_msgs::Odometry& goal, float32 timeSpent);
	void planCircle(nav_msgs::Odometry& goal, float32 timeSpent);
	void planIdentification(nav_msgs::Odometry& goal, float32 timeSpent);
	void planStraightLine(nav_msgs::Odometry& goal, float32 timeSpent);
	void planWaypoint(nav_msgs::Odometry& goal);

	bool m_isControlStarted;
	bool m_isFirstTimePass;

	float32 m_startTime;

	TrajectoryParameters m_parameters;
};
} // namespace DRONE


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
