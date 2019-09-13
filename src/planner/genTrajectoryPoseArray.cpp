/*
 * genTrajectory.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: João Benevides
 *      Modified: João Benevides - 07/Sep/2017
 */

#include "planner/genTrajectory.h"

namespace DRONE {

	Planner::Planner() {
		
		cout << "Starting Trajectory Planner ...\n" << endl;

		isControlStarted 	= false;
		PI 					= 3.141592653589793;
		firstTimePass 		= true;
   		t 			  		= 0.0;
		amplitude 	  		= 0.4;
		velMed        		= 0.5; //m/s
		wAng 		  		= 2*PI*velMed/(6.097*amplitude); // w = 2*pi*vel_media/(6.097*a);
		trajectory    		= 1;
		startTime 	  		= ros::Time::now().toSec();

		loadTopics(n);
		loadSettings(n);

		refreshWang(); //After parameters are changed, we need to recompute wAng

   		srand (time(NULL)); /* initialize random seed: */
	}

	Planner::~Planner () {

	}

	void Planner::setIsControlStarted(bool state){
		isControlStarted = state;
	}
	
	bool Planner::getIsControlStarted(void){
		return isControlStarted;
	}

	void Planner::loadTopics(ros::NodeHandle &n) {

		waypoint_publisher = n.advertise<geometry_msgs::PoseArray>("/drone/waypoint", 1);
		joy_subscriber 	   = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &Planner::joyCallback, this);
	}

	void Planner::loadSettings(ros::NodeHandle &n) {

		if (n.getParam("/drone/amplitude",amplitude)) {
			cout << "amplitude = " << amplitude << endl;
		}

		if (n.getParam("/drone/velMed",velMed)) {
			cout << "velMed = " << velMed << endl;
		}

		if (n.getParam("/drone/trajectory",trajectory)) {
			cout << "trajectory = " << trajectory << endl;
		}
	}

	void Planner::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
	  /*Enables "Automatic Control Mode" while pressing this button*/
	  if(joy->buttons[6]){
	    setIsControlStarted(true);
	  }
	  else{
		setIsControlStarted(false);
	  }
	}

	void Planner::refreshWang(void)
	{
		if(trajectory==0){
			wAng = 2*PI*velMed/(6.097*amplitude);
		}
		else if(trajectory==1){
			wAng = velMed/amplitude;
		}
	}

	void Planner::TrajPlanner(void)
	{
		geometry_msgs::PoseArray mGoal;
	    //Condition to redefine startTime each time select is pressed
	    if(getIsControlStarted()){
	    	if(firstTimePass){
	    		cout << "Starting Clock Now..." << endl;
	    		startTime = ros::Time::now().toSec();
				firstTimePass = false;
	    	}

	    	t = ros::Time::now().toSec() - startTime;


	    	if(trajectory == 0){
	  		
		  		//The Lemniscate of Gerono dictates that:
				mGoal.pose.pose.position.x = amplitude*sin(wAng*t);    //Was sin before.
				mGoal.pose.pose.position.y = 0.5*amplitude*sin(2*wAng*t);
			
			}
			else if(trajectory == 1){
			
				//The circle equation goes as:
				mGoal.pose.pose.position.x = amplitude*cos(wAng*t);    //Was sin before.
				mGoal.pose.pose.position.y = amplitude*sin(wAng*t);
			
			}
			else if(trajectory == 2){
				//The location of a point goes as:
				mGoal.pose.pose.position.x = amplitude;    //Was sin before.
				mGoal.pose.pose.position.y = amplitude;
			}
	    }
	    else
	    {
	    	firstTimePass = true;

			//set a random linear velocity in the x-axis
			mGoal.pose.pose.position.x = 0.0;
			mGoal.pose.pose.position.y = 0.0;
	    }

	    cout << "Current Time: " << t << endl;

		mGoal.pose.pose.position.z = 0.0;
		//set a random angular velocity in the y-axis
		mGoal.pose.pose.orientation.x = 0.0;
		mGoal.pose.pose.orientation.y = 0.0;
		mGoal.pose.pose.orientation.z = 0.0;
		mGoal.pose.pose.orientation.w = 1.0;

		//print the content of the message in the terminal
		//		ROS_INFO("[Random Walk] linear.x = %.2f, angular.z=%.2f\n", vel_msg.linear.x, vel_msg.angular.z);//

		//publish the message
		waypoint_publisher.publish(mGoal);
	}

	// void Planner::TrajPlanner(void)
	// {
	// 	geometry_msgs::PoseArray mGoal[];
	//     //Condition to redefine startTime each time select is pressed
	//     if(getIsControlStarted()){
	//     	if(firstTimePass){
	//     		cout << "Starting Clock Now..." << endl;
	//     		startTime = ros::Time::now().toSec();
	// 			firstTimePass = false;
	//     	}

	//     	t = ros::Time::now().toSec() - startTime;

	//     	//The Lemniscate of Gerono dictates that:
	// 		mGoal[0].position.x = amplitude*cos(wAng*t);    //Was sin before.
	// 		mGoal[0].position.y = 0.5*amplitude*sin(2*wAng*t);
	//     }
	//     else
	//     {
	//     	firstTimePass = true;

	// 		//set a random linear velocity in the x-axis
	// 		mGoal[0].position.x = 0.0;
	// 		mGoal[0].position.y = 0.0;
	//     }

	//     cout << "Current Time: " << t << endl;

	// 	mGoal[0].position.z = 0.0;
	// 	//set a random angular velocity in the y-axis
	// 	mGoal[0].orientation.x = 0.0;
	// 	mGoal[0].orientation.y = 0.0;
	// 	mGoal[0].orientation.z = 0.0;
	// 	mGoal[0].orientation.w = 1.0;


	// 	//Accelerations
	// 	mGoal[1].position.x = -wAng*wAng*mGoal[0].position.x;
	// 	mGoal[1].position.y = -4*wAng*wAng*mGoal[0].position.y;
	// 	mGoal[1].position.z = 0.0;
	// 	//set a random angular velocity in the y-axis
	// 	mGoal[1].orientation.x = 0.0;
	// 	mGoal[1].orientation.y = 0.0;
	// 	mGoal[1].orientation.z = 0.0;
	// 	mGoal[1].orientation.w = 1.0;

	// 	//print the content of the message in the terminal
	// 	//		ROS_INFO("[Random Walk] linear.x = %.2f, angular.z=%.2f\n", vel_msg.linear.x, vel_msg.angular.z);//

	// 	//publish the message
	// 	waypoint_publisher.publish(mGoal);
	// }

} //NAMESPACE DRONE


//Main Routine
int main(int argc, char **argv)
{
 
	ros::init(argc, argv, "genTrajectory");

 	try {

		DRONE::Planner wptNode;

		ros::Rate loop_rate(50);

		while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
	   	{
		    wptNode.TrajPlanner();
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
	   	}
		
		ros::spin();
	}
	catch (const std::exception &e) {
		ROS_FATAL_STREAM("An error has occurred: " << e.what());
		exit(1);
	}

  	return 0;
}

