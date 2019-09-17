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
		trajectory    		= "circle";
		startTime 	  		= ros::Time::now().toSec();
		poseDesired			= poseDesired.Zero();

		loadTopics(n);
		loadSettings(n);

		setTrajectoryCoefficients(); //Loads coefficients in case of straightLine

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

	VectorFive Planner::getposeDesired(void){
		
		return poseDesired;
	
	}

	void Planner::loadTopics(ros::NodeHandle &n) {

		waypoint_publisher = n.advertise<nav_msgs::Odometry>("/drone/waypoint", 1);
		joy_subscriber 	   = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &Planner::joyCallback, this);
	}

	void Planner::loadSettings(ros::NodeHandle &n) {

		if (n.getParam("/drone/amplitude",amplitude)) {
			cout << "amplitude = " << amplitude << endl;
		}

		if (n.getParam("/drone/velMed",velMed)) {
			cout << "velMed = " << velMed << endl;
		}

		string trajectory;
		if (n.getParam("/drone/trajectory",trajectory)) {
			setTrajectory(trajectory);
			cout << "trajectory = " << trajectory << endl;

		}
		
		vector<double> poseDesired;
		if (n.getParam("/drone/poseDesired",poseDesired)) {
			setposeDesired(VectorFive::Map(&poseDesired[0],5));
			cout << "poseDesired = " << getposeDesired() << endl;
		}		
	}

	void Planner::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	  /*Enables "Automatic Control Mode" while pressing this button*/
	  if(joy->buttons[6]){
	    setIsControlStarted(true);
	  }
	  else{
		setIsControlStarted(false);
	  }
	}

	void Planner::refreshWang(void){

		if(trajectory.compare("eightShape") == 0){
			wAng = 2*PI*velMed/(6.097*amplitude);
		}
		else { // should be if((trajectory.compare("circleXY") == 0)||(trajectory.compare("circleZXY") == 0))
			wAng = velMed/amplitude;
		}
	}
	
	void Planner::setposeDesired(VectorFive poseDesiredValue){
		
		poseDesired = poseDesiredValue;
	
	}

	void Planner::setTrajectory(const string& trajectoryInput){

		string DEFAULT_TRAJECTORY = "circleXY";

		if(trajectoryInput.compare("eightShape") == 0){
			trajectory = "eightShape";
		} else if(trajectoryInput.compare("circleXY") == 0){
			trajectory = "circleXY";
		} else if(trajectoryInput.compare("circleZXY") == 0){
			trajectory = "circleZXY";
		} else if(trajectoryInput.compare("ident") == 0){
			trajectory = "ident";
		} else if(trajectoryInput.compare("straightLine") == 0){
			trajectory = "straightLine";
		} else if(trajectoryInput.compare("wayPoint") == 0){
			trajectory = "wayPoint";
		} else {
			trajectory = DEFAULT_TRAJECTORY;
		}
	}

	void Planner::setTrajectoryCoefficients(void){
		double x, y, z, yaw, tf,tf3,tf4,tf5;
		x 		 = poseDesired(0);
		y 		 = poseDesired(1);
		z 		 = poseDesired(2);
		yaw 	 = poseDesired(3);
		tf 		 = poseDesired(4);
		tf3 	 = tf*tf*tf;
		tf4 	 = tf3*tf;
		tf5 	 = tf4*tf;
		
		cTx(0) 	 = 10*x/tf3;
		cTx(1) 	 = -15*x/tf4;
		cTx(2) 	 = 6*x/tf5;

		cTy(0) 	 = 10*y/tf3;
		cTy(1) 	 = -15*y/tf4;
		cTy(2) 	 = 6*y/tf5;

		cTz(0) 	 = 10*z/tf3;
		cTz(1) 	 = -15*z/tf4;
		cTz(2) 	 = 6*z/tf5;

		cTyaw(0) = 10*yaw/tf3;
		cTyaw(1) = -15*yaw/tf4;
		cTyaw(2) = 6*yaw/tf5;		

	}

	void Planner::angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll){
		// q = [w, x, y, z]'
		// Avaliar depois se vale a pena otimizar...
		//Verificado em 16/02/18
		q(0) = cos(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);
		q(1) = cos(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll) - sin(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll);
		q(2) = cos(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll);
		q(3) = sin(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) - cos(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);	
	}	

	void Planner::TrajPlanner(void)
	{
		nav_msgs::Odometry mGoal;
		float t2,t3,t4,t5;
		double yaw_desired; //, velZ;
		// int nVoltas = 6;
		// velZ = 1.2*velMed/(2*PI*amplitude*nVoltas);
		VectorQuat quatDesired;
		quatDesired = quatDesired.Zero();

	    //Condition to redefine startTime each time select is pressed
	    if(getIsControlStarted()){
	    	if(firstTimePass){
	    		cout << "Starting Clock Now..." << endl;
	    		startTime = ros::Time::now().toSec();
				firstTimePass = false;
	    	}

	    	t = ros::Time::now().toSec() - startTime;

	    	cout << "Select is pressed" << endl;

	    	if(trajectory.compare("eightShape") == 0){
				
	    		cout << "8-shape - Trajectory" << endl;

		  		//The Lemniscate of Gerono dictates that:
				mGoal.pose.pose.position.x = amplitude*sin(wAng*t);
				mGoal.pose.pose.position.y = 0.5*amplitude*sin(2*wAng*t);
				mGoal.pose.pose.position.z = 0.0;

				mGoal.twist.twist.linear.x = wAng*amplitude*cos(wAng*t);
				mGoal.twist.twist.linear.y = wAng*amplitude*cos(2*wAng*t);
				mGoal.twist.twist.linear.z = 0.0;

				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;

			
			}
			else if(trajectory.compare("circleXY") == 0){

				cout << "circle XY - Trajectory" << endl;
			
				//The circle equation goes as:
				mGoal.pose.pose.position.x = amplitude*cos(wAng*t);    
				mGoal.pose.pose.position.y = amplitude*sin(wAng*t);
				mGoal.pose.pose.position.z = 0;

				mGoal.twist.twist.linear.x = -wAng*amplitude*sin(wAng*t);
				mGoal.twist.twist.linear.y = wAng*amplitude*cos(wAng*t);
				mGoal.twist.twist.linear.z = 0;

				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;

				// yaw_desired = angles::normalize_angle(wAng*t);
				
				// angle2quatZYX(quatDesired, yaw_desired, 0.0 , 0.0);

				// mGoal.pose.pose.orientation.w = quatDesired(0);
				// mGoal.pose.pose.orientation.x = quatDesired(1);
				// mGoal.pose.pose.orientation.y = quatDesired(2);
				// mGoal.pose.pose.orientation.z = quatDesired(3);

				// mGoal.twist.twist.angular.x    = 0.0;
				// mGoal.twist.twist.angular.y    = 0.0;
				// mGoal.twist.twist.angular.z    = wAng;					
			
			}
			else if(trajectory.compare("ident") == 0){

				cout << "Ident - Trajectory" << endl;
			

				if(t < 40)
				{

					//The circle equation goes as:
					mGoal.pose.pose.position.x = 0.5*amplitude*(sin(wAng*t)+sin(t));    
					mGoal.pose.pose.position.y = 0.5*amplitude*(sin(wAng*t)+sin(t)); 
					mGoal.pose.pose.position.z = 0;

					mGoal.twist.twist.linear.x = -0.5*amplitude*(wAng*sin(wAng*t)+sin(t));
					mGoal.twist.twist.linear.y = 0.5*amplitude*(wAng*cos(wAng*t)+cos(t));
					mGoal.twist.twist.linear.z = 0;

					mGoal.pose.pose.orientation.x = 0.0;
					mGoal.pose.pose.orientation.y = 0.0;
					mGoal.pose.pose.orientation.z = 0.0;
					mGoal.pose.pose.orientation.w = 1.0;

					mGoal.twist.twist.angular.x    = -0.5*amplitude*(wAng*wAng*cos(wAng*t)+cos(t));
					mGoal.twist.twist.angular.y    = -0.5*amplitude*(wAng*wAng*sin(wAng*t)+sin(t));
					mGoal.twist.twist.angular.z    = 0.0;

				}
				else if(t < 80){
										
					mGoal.pose.pose.position.x = 0;    
					mGoal.pose.pose.position.y = 0;
					mGoal.pose.pose.position.z = 0.5*amplitude*(sin(wAng*t)+sin(t));

					mGoal.twist.twist.linear.x = 0;
					mGoal.twist.twist.linear.y = 0;
					mGoal.twist.twist.linear.z = -0.5*amplitude*(wAng*sin(wAng*t)+sin(t));

					mGoal.pose.pose.orientation.x = 0.0;
					mGoal.pose.pose.orientation.y = 0.0;
					mGoal.pose.pose.orientation.z = 0.0;
					mGoal.pose.pose.orientation.w = 1.0;

					mGoal.twist.twist.angular.x    = 0;
					mGoal.twist.twist.angular.y    = 0;
					mGoal.twist.twist.angular.z    = -0.5*amplitude*(wAng*wAng*cos(wAng*t)+cos(t));

				}

				else{ //GAMBIARRA EXTREMA! CUIDADO!
										
					mGoal.pose.pose.position.x = 0;    
					mGoal.pose.pose.position.y = 0;
					mGoal.pose.pose.position.z = 0;

					mGoal.twist.twist.linear.x = 0;
					mGoal.twist.twist.linear.y = 0;
					mGoal.twist.twist.linear.z = 0;

					yaw_desired = angles::normalize_angle(0.5*1.05*(cos(wAng*t)+cos(t)));
					
					angle2quatZYX(quatDesired, yaw_desired, 0.0 , 0.0);

					cout << "Enviando yaw_desired:" << yaw_desired << endl;

					mGoal.pose.pose.orientation.w = quatDesired(0);
					mGoal.pose.pose.orientation.x = quatDesired(1);
					mGoal.pose.pose.orientation.y = quatDesired(2);
					mGoal.pose.pose.orientation.z = quatDesired(3);

					mGoal.twist.twist.angular.x    = -0.5*1.05*(wAng*wAng*cos(wAng*t)+cos(t));
					mGoal.twist.twist.angular.y    = 0.0;
					mGoal.twist.twist.angular.z    = -0.5*1.05*(wAng*sin(wAng*t)+sin(t));

				}							
			}			
			else if(trajectory.compare("circleZXY") == 0){

				cout << "circle ZXY- Trajectory" << endl;
				/*TRANSLAÇÃO NORMAL*/
			
				// The circle equation goes as:
				mGoal.pose.pose.position.x = amplitude*cos(wAng*t);    
				mGoal.pose.pose.position.y = amplitude*sin(wAng*t);
				mGoal.pose.pose.position.z = amplitude*sin(wAng*t);

				mGoal.twist.twist.linear.x = -wAng*amplitude*sin(wAng*t);
				mGoal.twist.twist.linear.y = wAng*amplitude*cos(wAng*t);
				mGoal.twist.twist.linear.z = wAng*amplitude*cos(wAng*t);				

				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;				

				/*GIRO PURO*/

				// mGoal.pose.pose.position.x = 0.0;    
				// mGoal.pose.pose.position.y = 0.0;
				// mGoal.pose.pose.position.z = 0.0;

				// mGoal.twist.twist.linear.x = 0.0;
				// mGoal.twist.twist.linear.y = 0.0;
				// mGoal.twist.twist.linear.z = 0.0;				



				// yaw_desired = angles::normalize_angle(wAng*t);
				
				// angle2quatZYX(quatDesired, yaw_desired, 0.0 , 0.0);

				// cout << "Enviando yaw_desired:" << yaw_desired << endl;

				// mGoal.pose.pose.orientation.w = quatDesired(0);
				// mGoal.pose.pose.orientation.x = quatDesired(1);
				// mGoal.pose.pose.orientation.y = quatDesired(2);
				// mGoal.pose.pose.orientation.z = quatDesired(3);
				

				// mGoal.twist.twist.angular.x    = 0.0;
				// mGoal.twist.twist.angular.y    = 0.0;
				// mGoal.twist.twist.angular.z    = wAng;				
			
			}
			else if(trajectory.compare("straightLine") == 0){

				cout << "straightLine - Trajectory" << endl;
				if(t <= poseDesired(4)){
					
					t2 = t*t;
					t3 = t2*t;
					t4 = t3*t;
					t5 = t4*t;
					
					mGoal.pose.pose.position.x = cTx(0)*t3 + cTx(1)*t4 + cTx(2)*t5; 
					mGoal.pose.pose.position.y = cTy(0)*t3 + cTy(1)*t4 + cTy(2)*t5;
					mGoal.pose.pose.position.z = cTz(0)*t3 + cTz(1)*t4 + cTz(2)*t5;

					mGoal.twist.twist.linear.x = 3*cTx(0)*t2 + 4*cTx(1)*t3 + 5*cTx(2)*t4; 
					mGoal.twist.twist.linear.y = 3*cTy(0)*t2 + 4*cTy(1)*t3 + 5*cTy(2)*t4;
					mGoal.twist.twist.linear.z = 3*cTz(0)*t2 + 4*cTz(1)*t3 + 5*cTz(2)*t4;

				} else {

					mGoal.pose.pose.position.x = poseDesired(0); 
					mGoal.pose.pose.position.y = poseDesired(1);
					mGoal.pose.pose.position.z = poseDesired(2);

					mGoal.twist.twist.linear.x = 0; 
					mGoal.twist.twist.linear.y = 0;
					mGoal.twist.twist.linear.z = 0;
				}
				
				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;

				// mGoal.pose.pose.position.x = poseDesired(0); 
				// mGoal.pose.pose.position.y = poseDesired(1);
				// mGoal.pose.pose.position.z = poseDesired(2);

				// mGoal.twist.twist.linear.x = 0; 
				// mGoal.twist.twist.linear.y = 0;
				// mGoal.twist.twist.linear.z = 0;			
				
			}
			else if(trajectory.compare("wayPoint") == 0){

				cout << "wayPoint - Trajectory" << endl;
				
				// Position Desired
				mGoal.pose.pose.position.x = poseDesired(0); 
				mGoal.pose.pose.position.y = poseDesired(1);
				mGoal.pose.pose.position.z = poseDesired(2);

				// Linear Velocity Desired
				mGoal.twist.twist.linear.x = 0; 
				mGoal.twist.twist.linear.y = 0;
				mGoal.twist.twist.linear.z = 0;

				// Quaternioin Orientation Desired
				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				// Angular Velocity Desired
				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;

				// mGoal.pose.pose.position.x = poseDesired(0); 
				// mGoal.pose.pose.position.y = poseDesired(1);
				// mGoal.pose.pose.position.z = poseDesired(2);

				// mGoal.twist.twist.linear.x = 0; 
				// mGoal.twist.twist.linear.y = 0;
				// mGoal.twist.twist.linear.z = 0;			
				
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

