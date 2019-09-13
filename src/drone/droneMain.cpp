/*
 * droneMain.cpp
 *
 * Created by: rsinoue on 16.Jun.2017
 * Last Modified: jrsbenevides
 *
 * Description: This is the main function that maintains the controller running at X Hz, 
 *				where X is assigned in loop_rate(X).
 *
 *
 */

#include "drone/droneSystem.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_estimation");

	try {
		DRONE::System node; 

		ros::Rate loop_rate(100); // Defines update rate
		while (ros::ok())
		{
			node.control();  //Main control function
		    ros::spinOnce(); 
		    loop_rate.sleep(); 
		}
		ros::spin();
	}
	catch (const std::exception &e) {
		ROS_FATAL_STREAM("An error has occurred: " << e.what());
		exit(1);
	}

  	return 0;
}
