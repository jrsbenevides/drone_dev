/*
 * droneSystem.cpp
 *
 *  Created on: 16/06/2017
 *      Author: rsinoue
 *      Modified: João Benevides - 07/Sep/2017
 */

// System Class

#include "drone/droneSystem.h"

namespace DRONE {

	System::System() {

		initDroneParam();	

		loadTopics(n);
		
		loadSettings(n);

	}

	System::~System () {

	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 SETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/



	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setTrajectory
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'trajectory' global variable the one defined in config parameters file.
	*				  (Notice that, in case name was wrongly assigned, DEFAULT_TRAJECTORY will be chosen instead)
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void System::setTrajectory(const string& trajectoryInput){

		string DEFAULT_TRAJECTORY = "circleXY";

		if(trajectoryInput.compare("eightShape") == 0){
			trajectory = "eightShape";
		} else if(trajectoryInput.compare("circleXY") == 0){
			trajectory = "circleXY";
		} else if(trajectoryInput.compare("circleZXY") == 0){
			trajectory = "circleZXY";
		} else if(trajectoryInput.compare("straightLine") == 0){
			trajectory = "straightLine";
		} else if(trajectoryInput.compare("ident") == 0){
			trajectory = "ident";	
		} else if(trajectoryInput.compare("wayPoint") == 0){
			trajectory = "wayPoint";
		} else {
			trajectory = DEFAULT_TRAJECTORY;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setControlSelect
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'controlSelect' global variable the one setted in config parameters file.
	*				  (Notice that, in case name was wrongly assigned, DEFAULT_CONTROLLER will be chosen instead)
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void System::setControlSelect(const string& controlSelectInput){

		string DEFAULT_CONTROLLER = "PID";

		if(controlSelectInput.compare("PID") == 0){
			controlSelect = "PID";
		} else if(controlSelectInput.compare("FL") == 0){
			controlSelect = "FL";
		} else if(controlSelectInput.compare("RLQR") == 0){
			controlSelect = "RLQR";
		} else if(controlSelectInput.compare("SLQR") == 0){
			controlSelect = "SLQR";			
		} else if(controlSelectInput.compare("RecursiveLQR") == 0){
			controlSelect = "RecursiveLQR";			
		} else {
			controlSelect = DEFAULT_CONTROLLER;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setSensorSelect
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'sensorSelect' global variable the one setted in config parameters file.
	*				  (Notice that, in case name was wrongly assigned, DEFAULT_SENSOR will be chosen instead)
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::setSensorSelect(const string& sensorSelectInput){

		string DEFAULT_SENSOR = "IMU";

		if(sensorSelectInput.compare("IMU") == 0){
			sensorSelect = "IMU";
		} else if(sensorSelectInput.compare("VICON") == 0){
			sensorSelect = "VICON";
		} else {
			sensorSelect = DEFAULT_SENSOR;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setAmplitude
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'amplitude' global variable the one setted in config parameters file.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::setAmplitude(const double& amplitudeValue) {
		amplitude = amplitudeValue;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setVelMed
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'velMed' global variable the one setted in config parameters file.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	void System::setVelMed(const double& velMedValue) {
		velMed = velMedValue;
	}


	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #####################################            REGULAR FUNCTIONS                 ########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: initDroneParam
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Initialize essential functions for drone operation;
	*				  2. Initialize parameters and default values;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void System::initDroneParam(void){

		cout << "Starting Drone Node" << endl;

		// Sets flag in order to halt Vicon acquisition and make sure it will be properly initialized once/if called.
		drone.setIsViconStarted(false);
		// Sets initial time
		drone.setTimeNow(0);


		//Starting or Default Values
		count 	   		= 0;
		countEKF		= 1;
		PI 		 		= 3.141592653589793;
		flagEnable 		= 0;
		vxAmpl 			= 0;
		vyAmpl 			= 0;
		vzAmpl 			= 0;
		angAmpl 		= 0;
		f 				= 0;
		amplitude 		= 0.8;
		velMed      	= 0.1; //m/s
		trajectory		= "circleXY";
		controlSelect	= "PID";
		flagTwist 		= true;

		wAng 			= velMed/amplitude;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: load Topics
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Define the ROS Topics and its types of messages;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::loadTopics(ros::NodeHandle &n) {
		cmd_vel_publisher 		 = n.advertise<geometry_msgs::Twist>("/drone/cmd_vel",1);
		transfPosition_publisher = n.advertise<nav_msgs::Odometry>("/drone/transf_position",1);
		joy_subscriber 			 = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &System::joyCallback, this);
		odom_subscriber 		 = n.subscribe<nav_msgs::Odometry>("/drone/odom", 1, &System::odomCallback, this);
		waypoint_subscriber 	 = n.subscribe<nav_msgs::Odometry>("/drone/waypoint", 1, &System::waypointCallback, this);
		vicon_subscriber 	 	 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/bebop/bebop", 1, &System::viconCallback, this);
//		fix_subscriber = n.subscribe<sensor_msgs::NavSatFix>("/drone/fix", 10, &System::fixCallback, this);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: load Settings 
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Loads config parameters and loads them into the program by substituting previously created variables.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::loadSettings(ros::NodeHandle &n) {

		if (n.getParam("/drone/vxAmpl",vxAmpl)) {
			cout << "vxAmpl = " << vxAmpl << endl;
		}

		if (n.getParam("/drone/vyAmpl",vyAmpl)) {
			cout << "vyAmpl = " << vyAmpl << endl;
		}

		if (n.getParam("/drone/vzAmpl",vzAmpl)) {
			cout << "vzAmpl = " << vzAmpl << endl;
		}

		if (n.getParam("/drone/angAmpl",angAmpl)) {
			cout << "angAmpl = " << angAmpl << endl;
		}
		if (n.getParam("/drone/f",f)) {
			cout << "f = " << f << endl;
		}

		int isEKFonline;
		if (n.getParam("/drone/isEKFonline",isEKFonline)) {
			drone.setIsEKFonline(isEKFonline);
			cout << "isEKFonline = " << drone.getIsEKFonline() << endl;
		} else
		{
			ROS_ERROR("Failed to get param 'isEKFonline'");
		}

		int updateRateEKF;
		if (n.getParam("/drone/updateRateEKF",updateRateEKF)) {
			drone.setUpdateRateEKF(updateRateEKF);
			cout << "updateRateEKF = " << drone.getUpdateRateEKF() << endl;
		} else
		{
			ROS_ERROR("Failed to get param 'updateRateEKF'");
		}

		vector<double> K;
		if (n.getParam("/drone/K",K)) {
			drone.setK(Vector8d::Map(&K[0],8));
			cout << "K = [" << drone.getK().transpose() << " ]" << endl;

		}

		vector<double> Kstart;
		if (n.getParam("/drone/Kstart",Kstart)) {
			drone.setKstart(Vector8d::Map(&Kstart[0],8));
			cout << "Kstart = [" << drone.getKstart().transpose() << " ]" << endl;

		}

		vector<double> Kp;
		if (n.getParam("/drone/Kp",Kp)) {
			Matrix4d KpMatrix;
			KpMatrix = KpMatrix.Zero();
			KpMatrix.diagonal() = Vector4d::Map(&Kp[0],4);
			drone.setKp(KpMatrix);
			cout << "Kp = [" <<  drone.getKp() << " ]" << endl;

		}

		vector<double> Kd;
		if (n.getParam("/drone/Kd",Kd)) {
			Matrix4d KdMatrix;
			KdMatrix = KdMatrix.Zero();
			KdMatrix.diagonal() = Vector4d::Map(&Kd[0],4);
			drone.setKd(KdMatrix);
			cout << "Kd = [" << drone.getKd() << " ]" << endl;

		}

		vector<double> Ki;
		if (n.getParam("/drone/Ki",Ki)) {
			Matrix4d KiMatrix;
			KiMatrix = KiMatrix.Zero();
			KiMatrix.diagonal() = Vector4d::Map(&Ki[0],4);
			drone.setKi(KiMatrix);
			cout << "Ki = [" << drone.getKi() << " ]" << endl;

		}

		vector<double> threshold;
		if (n.getParam("/drone/threshold",threshold)) {
			drone.setThreshold(Vector4d::Map(&threshold[0],4));
			cout << "INICIALIZANDO THRESHOLD COMO = " << drone.getThreshold() << endl;
		}

		vector<double> inputRange;
		if (n.getParam("/drone/inputRange",inputRange)) {
			drone.setInputRange(Vector4d::Map(&inputRange[0],4));
			cout << "inputRange = " << drone.getInputRange() << endl;
		}
		
		double amplitude;
		if (n.getParam("/drone/amplitude",amplitude)) {
			setAmplitude(amplitude);
			cout << "Amplitude = " << amplitude << endl;
		}

		double velMed;
		if (n.getParam("/drone/velMed",velMed)) {
			setVelMed(velMed);
			cout << "Vel. Media = " << velMed << endl;
		}

		string trajectory;
		if (n.getParam("/drone/trajectory",trajectory)) {
			setTrajectory(trajectory);
			cout << "trajectory = " << trajectory << endl;

		}

		string controlSelect;
		if (n.getParam("/drone/controlSelect",controlSelect)) {
			setControlSelect(controlSelect);
			cout << "controlSelect = " << controlSelect << endl;
		}

		string sensorSelect;
		if (n.getParam("/drone/sensorSelect",sensorSelect)) {
			setSensorSelect(sensorSelect);
			cout << "sensorSelect = " << sensorSelect << endl;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: boot Vicon
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Receives a value of time ("timeValue") as an argument;
	*				  2. Verifies if selected sensor is set to "VICON";
	*				  3. Sets the given timeValue as timeOrigin;
	*				  4. Resets flag in order to reset coordinate frame.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::bootVicon(const double& timeValue){
		if(sensorSelect.compare("VICON") == 0){
			
			// while(!drone.getIsViconStarted()); 		// Waits for Vicon node to start publishing - Não funcionou!
			drone.setTimeOrigin(timeValue);
			drone.setIsOdomStarted(false);		   	// Uses current parameter to update pose next time viconCallback is called (same as down button)
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: control
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. This is the main control function. It depends on flagEnable (select button to run);
	*				  2. Gets current position;
	*				  3. Checks which controller to use. In case of PID, a reset of integral error is necessary and 
	*					 flagControllerStarted needs to be handled;
	*				  4. Gets the provided input and sets it to publish.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Função de Habilitação - Controle/Joystick
	void System::control() {
	  geometry_msgs::Twist cmd_vel_msg;
	  Vector3axes 	position;
	  Vector4d		xTemp;
	  
	  drone.setIsFlying(flagEnable); // 

	  if(flagEnable == 1) // Flag de habilitação do controlador
	  {

		position = drone.getPosition();

//		 Vector3axes positionDesired;
//
//		 positionDesired(0) = 0;
//		 positionDesired(1) = 0;
//		 positionDesired(2) = 0; //position(2);
//
//		 double yawDesired;
//
//		 yawDesired = 0;  // drone.getYaw()
//
//
//		 drone.setPositionDesired(positionDesired);
//		 drone.setYawDesired(yawDesired);


		Vector4d input; // Local Variable

		if(controlSelect.compare("PID") == 0){

			cout << "### PID ###" << endl;
			
			if(flagControllerStarted == true){ // Flag de confirmação do controlador "once time"
		  		drone.setXIntError(xTemp.Zero()); // Zera o erro integral
		  		flagControllerStarted = false;
	  		}
		
		 	input = drone.getPIDControlLaw();
		
		} else if(controlSelect.compare("FL") == 0){

			cout << "### Feedback Linearization ###" << endl;
			
			input = drone.getFLControlLaw();

		} else if(controlSelect.compare("RLQR") == 0){

			cout << "### Robust LQR ###" << endl;

			input = drone.getRobustControlLaw();

		} else if(controlSelect.compare("SLQR") == 0){

			cout << "### Standard LQR ###" << endl;

			input = drone.getLQRControlLaw();

		} else if(controlSelect.compare("RecursiveLQR") == 0){

			cout << "### Recursive LQR ###" << endl;

			input = drone.getRecursiveLQRControlLaw();

		} else {

			cout << "\n ERROR: Please select a valid controller" << endl;

		}

		 cout << "inputCL = [" << input.transpose() << "]" << endl;

		 // Load message var type
	     cmd_vel_msg.linear.x  = input(0);
	     cmd_vel_msg.linear.y  = input(1);
	     cmd_vel_msg.linear.z  = input(2);
	     cmd_vel_msg.angular.x = 0;            
	     cmd_vel_msg.angular.y = 0;            
	     cmd_vel_msg.angular.z = input(3);

	     // PUBLISH MESSAGE
	     cmd_vel_publisher.publish(cmd_vel_msg);
	     if(drone.getIsEKFonline()){
	     	drone.setCmdVel(input);
	     	if((countEKF % drone.getUpdateRateEKF()) == 0){
	     		drone.setEKFToken(true); //Enables EKF computing every 10 iterations of control.
	     		countEKF = 0;	
	     	}
	     	countEKF = countEKF + 1;
	     }
	  }
	  else{

	  	flagControllerStarted = true;

	  }
	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #######################################                 CALLBACKS                 #########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/



	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: joyCallback
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Unused buttons were used to special functions. These features are described below:
	*				  1. Button [14] = 'DIGITAL DOWN' = is responsible for resetting flag isOdomStarted. It allows the resetting of local
						 frame;
	*				  2. Button [6]  = 'SELECT' =  when kept pressed, allows controller to run.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
	{
      // geometry_msgs::Twist twist;

      /*Command to reset frame location*/
      if (joy->buttons[14]) {
		drone.setIsOdomStarted(false);
      } //Não seria melhor por uma condição de falso aqui para subir o flag de novo?

      /*Enables "Automatic Control Mode" while pressing this button*/
	  if(joy->buttons[6]){
	    flagEnable = 1;
	  }
	  else{
	  	flagEnable = 0;
	  }
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: odomCallback
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. It is enabled only when 'IMU' is selected as sensor;
	*				  2. Gets current time, position, orientation, linear and angular velocity;
	*				  3. Uses current position and orientation to set new coordinate frame in case that flag allows it do to so;
	*				  4. Sets time, position, orientation, linear and angular velocity in local terms;
	*				  5. Gets current local position and orientation in order to call function globalToLocalPosition.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)	{
		if(sensorSelect.compare("IMU") == 0){
			/*Position and Linear Velocity are delivered with respect to global frame*/
			/*angularVel(Ang Velocity) is same for both local and global frame*/
			Vector3axes position, positionLocal, angularVel, linearVel, rpy;
			/*Orientation is same for both local and global frame*/
			VectorQuat orientation, orientationLocal;

			double timeNow, yawOdom;

			position 	<< odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z;
			linearVel 	<< odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;
			orientation << odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z;
			angularVel 	<< odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z;
			
			timeNow = odom->header.stamp.toSec();

			/*Reset frame location*/
			if (!drone.getIsOdomStarted()) {
				Conversion::quat2angleZYX(rpy,orientation);
				yawOdom = angles::normalize_angle(rpy(2));				//Added normalize...check if it works!!!
				drone.setPosition0(position,yawOdom);
			}
			
			drone.setPosition(position);
			drone.setOrientation(orientation);
			drone.setAngularVel(angularVel);
			drone.setLinearVel(linearVel);
			drone.setTimeNow(timeNow);
			
			cout << "Pose updated (IMU)" << endl;
			cout << "angular Velocity: twist: " << angularVel.transpose() << endl;

			/*Envia mensagens de position corrigidas toda vez que uma nova mensagem de odom chega*/
			/*nav_msgs::Odometry*/

			positionLocal 	 = drone.getPosition();
			orientationLocal = drone.getOrientation();

			globalToLocalPosition(positionLocal, orientationLocal, linearVel, angularVel);
		}

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: globalToLocal
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets position and orientation values (local) and publishes them in a topic in order to be later
	*				  	 assessed by the plotControl node.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::globalToLocalPosition(const Vector3axes& positionValue, const VectorQuat& orientationValue, const Vector3axes& linearVelValue,const Vector3axes& angularVelValue){

		nav_msgs::Odometry goalPosition;

		goalPosition.pose.pose.position.x 		= positionValue(0);
		goalPosition.pose.pose.position.y 		= positionValue(1);
		goalPosition.pose.pose.position.z 		= positionValue(2);

		goalPosition.pose.pose.orientation.w 	= orientationValue(0);				
		goalPosition.pose.pose.orientation.x 	= orientationValue(1);				
		goalPosition.pose.pose.orientation.y 	= orientationValue(2);				
		goalPosition.pose.pose.orientation.z 	= orientationValue(3);

		goalPosition.twist.twist.linear.x 		= linearVelValue(0);
		goalPosition.twist.twist.linear.y 		= linearVelValue(1);
		goalPosition.twist.twist.linear.z 		= linearVelValue(2);						
		
		goalPosition.twist.twist.angular.x 		= angularVelValue(0);
		goalPosition.twist.twist.angular.y 		= angularVelValue(1);
		goalPosition.twist.twist.angular.z 		= angularVelValue(2);

		transfPosition_publisher.publish(goalPosition);

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: waypointCallback
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets position and velocity desired;
	*				  2. Because acceleration does not come in the message we need to calculate it in order to compute it
	*					 in case of eightShape and circle trajectories;
	*				  3. Checks which trajectory was chosen and computes acceleration accordingly;
	*				  4. Sets desired position, orientation, velocity and acceleration. (Notice: with respect to local frame).
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::waypointCallback(const nav_msgs::Odometry::ConstPtr& waypoint){
		
		Vector3axes eulerAngles,positionToGo, dpositionToGo, d2positionToGo;
		VectorQuat quatAngles;
		// Vector4d pastPositionToGo, pastVelocityToGo;

		double yawAngle, yawDesired, dYawDesired, d2YawDesired;

		positionToGo <<  waypoint->pose.pose.position.x,
						 waypoint->pose.pose.position.y,
						 waypoint->pose.pose.position.z;

		dpositionToGo << waypoint->twist.twist.linear.x,
						 waypoint->twist.twist.linear.y,
						 waypoint->twist.twist.linear.z;


		if(trajectory.compare("eightShape") == 0){						 
		
			wAng = 2*PI*velMed/(6.097*amplitude);
			
			d2positionToGo << -wAng*wAng*(waypoint->pose.pose.position.x),
							  -4*wAng*wAng*(waypoint->pose.pose.position.y),
							  0;
			yawDesired  = 0.0;
		  	dYawDesired = 0.0;	
		  	d2YawDesired = 0.0;						  										  
		
		}
		else if(trajectory.compare("circleXY") == 0){
		
			wAng = velMed/amplitude;
			
			d2positionToGo << -wAng*wAng*(waypoint->pose.pose.position.x),
							  -wAng*wAng*(waypoint->pose.pose.position.y),
							  0;		

		    yawDesired  = 0.0;
		 	dYawDesired = 0.0;	
		 	d2YawDesired = 0.0;

		 //  	quatAngles 		<< 	waypoint->pose.pose.orientation.w,
			// 				  	waypoint->pose.pose.orientation.x,
			// 				  	waypoint->pose.pose.orientation.y,
			// 				  	waypoint->pose.pose.orientation.z;

			// // //Testar chamar em vez de quatAngles -> waypoint->pose.orientation
			// Conversion::quat2angleZYX(eulerAngles,quatAngles);

			// yawDesired  = eulerAngles(2);
		  	
		 //  	dYawDesired = waypoint->twist.twist.angular.z;						  					
		}		
		else if(trajectory.compare("circleZXY") == 0){
		
			wAng = velMed/amplitude;
			
			d2positionToGo << -wAng*wAng*(waypoint->pose.pose.position.x),
							  -wAng*wAng*(waypoint->pose.pose.position.y),											  
							  -wAng*wAng*(waypoint->pose.pose.position.z);				
			
			// d2positionToGo << 0.0,
			// 				  0.0,											  
			// 				  0.0;			
			
			quatAngles 		<< 	waypoint->pose.pose.orientation.w,
							  	waypoint->pose.pose.orientation.x,
							  	waypoint->pose.pose.orientation.y,
							  	waypoint->pose.pose.orientation.z;

			// //Testar chamar em vez de quatAngles -> waypoint->pose.orientation
			Conversion::quat2angleZYX(eulerAngles,quatAngles);

			yawDesired  = eulerAngles(2);
		  	
		  	dYawDesired = waypoint->twist.twist.angular.z;
		  	d2YawDesired = 0.0;

		}
		else if(trajectory.compare("ident") == 0){
			
			quatAngles 		<< 	waypoint->pose.pose.orientation.w,
							  	waypoint->pose.pose.orientation.x,
							  	waypoint->pose.pose.orientation.y,
							  	waypoint->pose.pose.orientation.z;

			Conversion::quat2angleZYX(eulerAngles,quatAngles);

			yawDesired  = eulerAngles(2);

			//Atenção: Gambiarra!
			if((yawDesired == 0)&&(flagTwist == true)){ //Significa que usamos o twist angular como aceleração linear desejada
				d2positionToGo << waypoint->twist.twist.angular.x,
							  	  waypoint->twist.twist.angular.y,											 
							  	  waypoint->twist.twist.angular.z;	

				dYawDesired = 0.0;

				// cout << "################################################ T < 60 ########################################" << endl;							  	  
			}
			else{ //Significa que usamos o twistangular z como vel ang desejada e twistangular x como acel ang desejada

				flagTwist = false;

				d2positionToGo << 0.0,
							  	  0.0,											 
							  	  0.0;	

				dYawDesired  = waypoint->twist.twist.angular.z;	
				d2YawDesired = waypoint->twist.twist.angular.x;	

				// cout << "################################################ YAAAAAW ########################################" << endl;
			}
		}
		else if(trajectory.compare("straightLine") == 0){

			wAng = velMed/amplitude;

			d2positionToGo << 	0,
							  	0,
							  	0;
			yawDesired  = 0.0;
		  	dYawDesired = 0.0;							  			
		  	d2YawDesired = 0.0;
		}
		else if(trajectory.compare("wayPoint") == 0){

			wAng = velMed/amplitude;

			d2positionToGo << 	0,
							  	0,
							  	0;
			yawDesired  = 0.0;
		  	dYawDesired = 0.0;							  			
		  	d2YawDesired = 0.0;
		}

		// cout << "positionDesired (local) = [ " << positionToGo.transpose() << " ] " <<  endl;

		drone.setPositionDesired(positionToGo);
		drone.setYawDesired(yawDesired);

		drone.setdPositionDesired(dpositionToGo);
		drone.setdYawDesired(dYawDesired);
		
		drone.setd2PositionDesired(d2positionToGo);
		drone.setd2YawDesired(d2YawDesired);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: viconCallback
	*	  Created by: jrsbenevides
	*  Last Modified: March 8th 2018 - jrsbenevides
	*
	*  	 Description: 1. It is enabled only when 'VICON' is selected as sensor;
	*				  2. Gets current time, position and orientation;
	*				  3. Sets position now, because we need to compute velocity;
	*				  4. Because Vicon sensor does not deliver either linear or angular velocity, we need estimate them through a
	*					 simple Kalman Filter, which is performed by DvKalman and DwKalman functions
	*				  5. Uses current position and orientation to set new coordinate frame in case that flag allows it do to so;
	*				  6. Sets orientation, linear and angular velocity in local terms;
	*				  7. Does the 'ViconBoot' when passing the callback for the first time;
	*				  8. Sets current time;
	*				  9. Gets current local position and orientation in order to call function globalToLocalPosition.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::viconCallback(const geometry_msgs::TransformStamped::ConstPtr& vicon){

		if(sensorSelect.compare("VICON") == 0){

			Vector3axes positionVicon, positionPast, positionNow, positionLocal, angularVel, linearVel, rpy;
			VectorQuat orientationVicon, orientationPast, orientationLocal,orientationNow;

			double time, timePast, yawVicon;

			positionPast 	= drone.getPosition(); 		//Stores last known position as past.
			orientationPast	= drone.getOrientation();	//Stores last known orientation as past.
			timePast 		= drone.getTimeNow(); 		//Stores last known time as past.

			time			= drone.getThisTime(vicon->header.stamp.toSec()); // returns time in sec after start
			
			// cout << "time: "  << time 	<< endl;
			
			positionVicon	<< vicon->transform.translation.x, vicon->transform.translation.y, vicon->transform.translation.z; //gets current position
			drone.setPosition(positionVicon);
			positionNow		= drone.getPosition();
			// linearVel 		= drone.getLinearVelVicon(positionNow,positionPast,time,timePast); //global terms instead of local (IMU)
			linearVel 		= drone.DvKalman(positionNow,time,timePast); //global terms instead of local (IMU)
			orientationVicon 	<< vicon->transform.rotation.w, vicon->transform.rotation.x, vicon->transform.rotation.y, vicon->transform.rotation.z;
			drone.setOrientation(orientationVicon); //ADD
			orientationNow	= drone.getOrientation();
			// angularVel 		= drone.getAngularVelVicon(orientationNow,orientationPast,time,timePast); //global terms instead of local (IMU)
			angularVel 		= drone.DwKalman(orientationNow,time,timePast);
			// cout << "\nPosition VICON:"	<< position.transpose() 	<< endl;
			// cout << "\nPosition Now:" 	<< positionNow.transpose() 	<< endl;
			// cout << "\nPosition Past:" 	<< positionPast.transpose() << endl;
			// cout << "\nOrientation:" 	<< orientation.transpose() 	<< endl;

			/*Reset frame location*/
			if (!drone.getIsOdomStarted()) {
				Conversion::quat2angleZYX(rpy,orientationVicon);
				yawVicon = rpy(2); //add normalize?
				drone.setPosition0(positionVicon,yawVicon);
				drone.setOrientation(orientationVicon);
			}
			
			drone.setAngularVel(angularVel);
			drone.setLinearVelVicon(linearVel); //Because Vicon velocity is obtained with respect to global coordinates we made another function to update values.
			
			// cout << "Pose updated (VICON)" << endl;

			if(!drone.getIsViconStarted()){
				bootVicon(time);
				drone.setIsViconStarted(true);
			}

			drone.setTimeNow(time); //Stores this time as current!

			/*Envia mensagens de position corrigidas toda vez que uma nova mensagem de odom chega*/
			/*nav_msgs::Odometry*/

			globalToLocalPosition(positionNow, orientationNow,linearVel,angularVel);
		}

	}
} // namespace DRONE