/*
 * drone.cpp
 *
 * Created on: 08/07/2017
 *      Author: rsinoue
 *
 * Modified on: Sep 16th 2019
 *			by: jrsbenevides
 *
 * Description: Every function that belongs to the DRONE class.
 *
 *
 */


#include "drone/drone.h"

namespace DRONE {

	Drone::Drone() {

		initDroneParam();							  
	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 SETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Position
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides - Considering rotation and not only translation to describe the new frame.
	*
	*  	 Description: 1. Gets global position (either from IMU or VICON) and stores as local position;
	*				  2. Updates position error, because position has changed. Notice that position error is treated as local;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setPosition(const Vector3axes& positionValue) {
		position 		= RotGlobal.transpose()*(positionValue - position0);
		positionError 	= position - positionDesired;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Position Zero
		*	  Created by: rsinoue
	*  Last Modified: jrsbenevides - Flag reset (button enabled)
	*
	*  	 Description: 1. Gets a global position and defines it as the new translation vector from global to local coordinate frame;
	*				  2. Gets current global yaw angle and defines it as the initial angle of the new local coordinate frame. (Pitch and roll
	*					 angles are assumed to be zero for all effects);
	*				  3. Updates Rotation Matrix from global to local, because yaw0 has just changed;
	*				  4. Resets flag that enables function setPosition0 to run;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	void Drone::setPosition0(const Vector3axes& positionValue, const double& yawValue) { //change name to setFrame0
		position0 = positionValue;
		yaw0      = yawValue;

		RotGlobal.block<2,2>(0,0) << cos(yaw0), -sin(yaw0),
									 sin(yaw0),  cos(yaw0);

		setIsOdomStarted(true); /*Raises flag again*/
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Position Desired
		*	  Created by: rsinoue
	*  Last Modified: 
	*
	*  	 Description: 1. Gets a reference position and defines it as a desired position;
	*				  2. Updates potition error because position desired has changed;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setPositionDesired(const Vector3axes& positionValue) {
		positionDesired = positionValue;
		positionError 	= position - positionDesired;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set dPosition Desired
	*	  Created by: rsinoue
	*  Last Modified:
	*
	*  	 Description: 1. Gets a velocity vector and defines it as a desired velocity;
	*				  2. Updates velocity error because velocity desired has changed;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setdPositionDesired(const Vector3axes& dPositionValue) {
		dPositionDesired = dPositionValue;
		dPositionError 	 = dPosition - dPositionDesired;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set d2Position Desired
	*	  Created by: rsinoue
	*  Last Modified: 
	*
	*  	 Description: 1. Gets a acceleration vector and defines it as a desired acceleration;
	*				  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setd2PositionDesired(const Vector3axes& d2PositionValue) {
		d2PositionDesired = d2PositionValue;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Orientation
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Gets a quaternion vector and defines it as current orientation;
	*				  2. Converts the quaternion vector into euler angles, roll, pitch and yaw, with yaw normalization;
	* 				  3. Calculate the current yaw error;
	*				  4. Updates local rotation matrix;
	*				  5. Runs EKF (if enabled) for online estimation of K;
	*           	  5. Updates model matrices;
	* 				  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setOrientation (const VectorQuat& orientationValue){
		
		Vector3axes rpy, ddposNow, dposNow;
		Vector9d xFull, angFull;
		Vector4d z, qdg, qdb, v, h, y;
		Vector32x1 x;

		Matrix4d M,N,S;
		Matrix4x32 H;
		Matrix32x4 Kk;

		double sinyaw, cosyaw, ddyaw, dyaw;

		orientation = orientationValue;

	    Conversion::quat2angleZYX(rpy,orientation);

	    yaw = angles::normalize_angle(rpy(2)-yaw0);

		yawError = angles::normalize_angle(yaw - yawDesired);

		sinyaw = sin(yaw);
		cosyaw = cos(yaw);

		Rotation.block<2,2>(0,0) << cosyaw, -sinyaw,
				 	 	 	 	 	sinyaw,  cosyaw;

		F1 << K(0)*cosyaw, -K(2)*sinyaw, 	0,      0,
			  K(0)*sinyaw,  K(2)*cosyaw, 	0,      0,
			  			0,		  	  0, K(4),   	0,
			  			0,            0,    0,   K(6);

		F2 << K(1)*cosyaw, -K(3)*sinyaw, 	0,      0,
			  K(1)*sinyaw,  K(3)*cosyaw, 	0,      0,
			  			0,			  0, K(5),  	0,
			  			0,            0,    0,   K(7);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Linear Vel (IMU)
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides - Considering rotation and not only translation to describe the new frame.
	*
	*  	 Description: 1. Gets a linear velocity vector from IMU and stores it as 'dPosition' (because IMU provides local 
	*					 velocity information);
	*				  2. Computes velocity error based on local rotation matrix (because velocity error is defined globally);
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setLinearVel(const Vector3axes& linearVelValue) {
		Matrix3d 		Rot33;
		Rot33 			<< Rotation.block<3,3>(0,0);
	    dPosition 		= linearVelValue;
		dPositionError 	<< Rot33*dPosition - dPositionDesired;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Linear Vel Vicon
	*	  Created by: jrsbenevides on Feb.2018
	*  Last Modified: 
	*
	*  	 Description: 1. Gets a linear velocity vector from Vicon cameras and stores it as 'dPosition' after converting it back 
	*					 to local frame coordinates;
	*				  2. Computes velocity error (because velocity has changed);
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	void Drone::setLinearVelVicon(const Vector3axes& linearVelValue) {
		Matrix3d 		Rot33;
		Rot33 			<< Rotation.block<3,3>(0,0);
	    dPosition 		= Rot33.transpose()*linearVelValue;
		dPositionError 	<< linearVelValue - dPositionDesired;
	}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Angular Vel
	*	  Created by: rsinoue
	*  Last Modified:
	*
	*  	 Description: 1. Gets a angular velocity vector from IMU or Vicon cameras and stores it as 'angularvel';
	*				  2. Updates dyaw and computes its error;
	* 				  
	* 		  Issues: 1. IMU does not provide angular velocity information (a null vector is received instead);   		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setAngularVel(const Vector3axes& angularVelValue) {
		angularVel 	= angularVelValue;
	    dYaw 		= angularVel(2);
	    dYawError  	= dYaw - dYawDesired;
	}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set K
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets the model parameters vector (K1, K2, ..., K8);
	*				   		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setK(const Vector8d& KValues) {
		K = KValues;
	}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Kstart
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: 1. Starts the model parameters vector (K1, K2, ..., K8) to online EKF;
	*				   		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setKstart(const Vector8d& KstartValues) {
		Kstart = KstartValues;
	}	

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Yaw Desired
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides - Considering rotation and not only translation to describe the new frame.
	*
	*  	 Description: 1. Gets an angle value to be set as the yaw angle desired;
	*				  2. Updates yaw error;
	* 				    		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setYawDesired(const double& yawValue) {
		yawDesired = yawValue;
		yawError = angles::normalize_angle(yaw - yawDesired);
	}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set dYaw Desired
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides - Considering rotation and not only translation to describe the new frame.
	*
	*  	 Description: 1. Gets an angular velocity value to be set as the yaw angular velocity desired;
	*				  2. Updates dyaw error;
	* 				    		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setdYawDesired(const double& dYawValue) {
		dYawDesired = dYawValue;
	    dYawError  = dYaw - dYawDesired;
	}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set d2Yaw Desired
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Gets an angular acceleration value to be set as the yaw angular acceleration desired;
	*				  2. Updates d2yaw error;
	* 				    		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setd2YawDesired(const double& d2YawValue) {
		d2YawDesired = d2YawValue;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Kp
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets a 4x4 matrix and defines it as the proportional gain for the PID control/Feedback Linearization;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setKp(const Matrix4d& KpMatrix) {
		Kp = KpMatrix;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Kd
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets a 4x4 matrix and defines it as the derivative gain for the PID control/Feedback Linearization;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setKd(const Matrix4d& KdMatrix){
		Kd = KdMatrix;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Ki
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets a 4x4 matrix and defines it as the integral gain for the PID control/Feedback Linearization;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setKi(const Matrix4d& KiMatrix){
		Ki = KiMatrix;
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set XIntError
	*	  Created by: jrsbenevides on Feb. 2018
	*  Last Modified:
	*
	*  	 Description: 1. Updates current error integral for PID control;	    		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setXIntError(const Vector4d& xIntErrorValue){
		xIntError = xIntErrorValue;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Time Now
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets a time value (double) and sets it as current time;
	* 				  2. Computes 'deltaTime' based on previous computed time;
	* 				  3. Sets previous computed time as current time;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setTimeNow(const double& timeValue){
		
		timeNow 	= timeValue;
		deltaTime 	= timeNow - timePast;
		timePast  	= timeNow;
		
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Threshold
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets a threshold to the position and yaw error (Due to precision error, we decided to send a null 
	*					 input when absolute error is less than the pre-defined 'threshold' value); 
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setThreshold(const Vector4d& upperLimit) {
		threshold = upperLimit;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Input Range
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets the saturation range for the allowable inputs;	  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setInputRange(const Vector4d& inputRangeValue) {
		inputRange = inputRangeValue;
	}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Time Origin
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets the initialization time as 'timeOrigin' right after ViconCallback starts to acquire data;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setTimeOrigin(const  double& timeValue){
		timeOrigin = timeValue;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Is Odom Started
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets the flag "isOdomStarted" that controls frame coordinate reset, i.e. 
	*                    {x,y,z,phi,theta,psi} => {0,0,0,0,0,0};
	*       	P.S.: DEFAULT = 'true', i.e. a flag set as 'false' will reset the coordinate frame.  		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setIsOdomStarted(const bool& value) {
		isOdomStarted = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Is Vicon Started
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets the flag "isViconStarted" that controls first call of Vicon data;		    		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setIsViconStarted(const bool& value) {
		isViconStarted = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: set Is EKF online
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets the flag "isEKFonline" to enable/disable online EKF estimation for K parameters; 
	*					 This feature will be available in future versions only. DEFAULT = 'false'
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setIsEKFonline(const int& value) {
		isEKFonline = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setUpdateRateEKF
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. For future use only;  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setUpdateRateEKF(const int& value) {
		updateRateEKF = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setKalmanX
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets Kalman State Matrix for computing position;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setKalmanX(const Vector9d& value) {
		x_kalman = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setKalmanP
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets Kalman P Matrix - Position;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	void Drone::setKalmanP(const Matrix9d& value) {
		P_kalman = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setKalmanXAngular
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets Kalman State Matrix for computing angular speed;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	void Drone::setKalmanXAngular(const Vector9d& value) {
		xAng_kalman = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setKalmanPAngular
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets Kalman Matrix - Angular;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setKalmanPAngular(const Matrix9d& value) {
		PAng_kalman = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setXfromEKF
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. For future use only. This feature will be available in a future version;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setXfromEKF(const Vector32x1& value){
		x_EKF = value;
		Vector1x32 vectorDelta;
		vectorDelta << value.transpose();
		
		for(int i=0;i<4;i++){
			deltaM.block<1,4>(i,0) << vectorDelta.segment<4>(4*i);
			deltaN.block<1,4>(i,0) << vectorDelta.segment<4>(4*i+16);	
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setCmdVel
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Stores vector of control input. For future use only. This feature will be available in a future version;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setCmdVel(const Vector4d& value){
		cmdvelMsg = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setIsFlying
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. For future use only. This feature will be available in a future version;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setIsFlying(const bool& value){
		isDroneFlying = value;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setEKFToken
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. For future use only. This feature will be available in a future version;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::setEKFToken(const bool& value){
		tokenEKF = value;
	}
	

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 GETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: GETTERS
	*	  Created by: jrsbenevides/rsinoue
	*  Last Modified:
	*
	*  	 Description: Below are the standard getters methods;		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	bool Drone::getIsOdomStarted(void){

		return isOdomStarted;
	}

	bool Drone::getIsViconStarted(void){

		return isViconStarted;
	}

	int Drone::getIsEKFonline(void){

		return isEKFonline;
	}

	int Drone::getUpdateRateEKF(void){

		return updateRateEKF;
	}

	double Drone::getTimeNow(void){
		return timeNow;
	}	

	Vector3axes Drone::getPosition(void) {
		return position;
	}

	VectorQuat Drone::getOrientation(void) {
		return orientation;
	}

	double Drone::getRoll(void) {
		return roll;
	}

	double Drone::getPitch(void) {
		return pitch;
	}

	double Drone::getYaw(void) {
		return yaw;
	}

	Vector3axes Drone::getRPY(void) {
		return rpy;
	}

	Vector8d Drone::getK(void) {
		return K;
	}

	Vector8d Drone::getKstart(void) {
		return Kstart;
	}

	Matrix4d Drone::getF1(void) {
		return F1;
	}

	Matrix4d Drone::getF2(void) {
		return F2;
	}

	Matrix4d Drone::getKp(void) {
		return Kp;
	}
	
	Matrix4d Drone::getKd(void){
		return Kd;
	}

	Matrix4d Drone::getKi(void){
		return Ki;
	}

	Vector4d Drone::getXIntError(void){
		return xIntError;
	}

	double Drone::getDeltaTimeNow(void){
		return deltaTime;
	}

	double Drone::getTimeOrigin(void){
		return timeOrigin;
	}

	Vector4d Drone::getThreshold(void){
		return threshold;
	}

	Vector4d Drone::getInputRange(void){
		return inputRange;
	}	

	Vector9d Drone::getKalmanX(void){
		return x_kalman;
	}

	Matrix9d Drone::getKalmanP(void){
		return P_kalman;
	}

	Vector9d Drone::getKalmanXAngular(void){
		return xAng_kalman;
	}

	Matrix9d Drone::getKalmanPAngular(void){
		return PAng_kalman;
	}

	Vector32x1 Drone::getXfromEKF(void){
		return x_EKF;
	}

	Vector4d Drone::getCmdVel(void){
		return cmdvelMsg;
	}

	bool Drone::getIsFlying(void){
		return isDroneFlying;
	}

	bool Drone::getEKFToken(void){
		return tokenEKF;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get This Time
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Get the time past subtracted from "getTimeOrigin", i.e., current time (in sec) since the beginning (zero);
	* 				    		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double Drone::getThisTime(const double& thisTime){
		return thisTime - getTimeOrigin();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get Linear Vel Vicon
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: Estimate linear velocity by simple differentiation (No use while using the Kalman Filter)
	*  	 	   Steps: 1. Receive as arguments the current position vector, the past position vector and its times;
	* 				  2. Compute the 'delta' position and the 'delta' time;
	*				  3. Compute the 'inverse of 'delta' time;	
	* 				  4. Compute the discrete derivative of position, called 'linearVelVicon' and return it;		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector3axes Drone::getLinearVelVicon(const Vector3axes& posCurrent, const Vector3axes& posPast, const double& timeNow, const double& timePast){
		
		double timeDiff, invTimeDiff;
		Vector3axes linearVelVicon, positionDiff;
		
		positionDiff	= posCurrent - posPast;
		timeDiff 		= timeNow - timePast;

		invTimeDiff = 1/timeDiff;

		linearVelVicon 	<<  invTimeDiff*positionDiff;
		
		return linearVelVicon;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get Angular Vel Vicon
	*	  Created by: jrsbenevides
	*  Last Modified:
	*  	 Description: Estimate angular velocity by simple differentiation (No use while using the Kalman Filter)
	*  	 	   Steps: 1. Receive as arguments the current quaternion vector, the past quaternion vector and its times;
	*                 2. Convert the 4x1 quaternions vectors to 3x1 euler angles vector;
	* 				  3. Compute the 'delta' time;	
	* 				  4. Compute the discrete angular velocity (by subtraction of current and past angle vectors), called 'angularVelVicon' and return it;		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector3axes Drone::getAngularVelVicon(const VectorQuat& orientCurrent, const VectorQuat& orientPast, const double& timeNow, const double& timePast){
		
		double timeDiff,angRoll,angPitch,angYaw;
		Vector3axes angularVelVicon,rpyCurrent, rpyPast;
		
	    Conversion::quat2angleZYX(rpyCurrent,orientCurrent);
	    Conversion::quat2angleZYX(rpyPast,orientPast);
		
		timeDiff 		= timeNow - timePast;
		angRoll 	= angles::normalize_angle(rpyCurrent(0) - rpyPast(0));
		angPitch 	= angles::normalize_angle(rpyCurrent(1) - rpyPast(1));
		angYaw 		= angles::normalize_angle(rpyCurrent(2) - rpyPast(2));
		angularVelVicon 	<<  angRoll,
								angPitch, 
								angYaw;
								
		angularVelVicon *= (1/timeDiff);							
		
		return angularVelVicon;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get Hinf Control Law
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: Computes the H-Infinite Control Law (For future versions only)
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector4d Drone::getHinfControlLaw (void) {

		Vector4d xError, xIntError;
		Vector4d dx;
		Vector4d dxError;
		Vector4d d2xDesired;

		double deltaTAtual;

		xError.head(3) = positionError;
		xError(3) = yawError;

		dx.head(3) = dPosition;
		dx(3) = dYaw;

		dxError.head(3) = dPositionError;
		dxError(3) = dYawError;

		d2xDesired.head(3) = d2PositionDesired;
		d2xDesired(3) = d2YawDesired;

		u = d2xDesired  + Kp*xError + Kd*dxError;

		input = F1.inverse()*(u + F2*dx);
		

		// NEXT COMMENTS: WAITING FOR APPROVAL => DELETE IF WORKS

		// for(int i=0; i < 4;i++){

		// 	if(abs(xError(i)) < threshold(i)){
		// 		input(i) = 0.0;
		// 	}

		// 	if(abs(input(i)) > inputRange(i)){
		// 		if(input(i) > 0){
		// 			input(i) = inputRange(i);
		// 		} else {
		// 			input(i) = -inputRange(i);
		// 		}
		// 	}
		// }

		return input;
	}	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get FL Control Law
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Define local variables and print the current position error, position desired and current postion;	
	* 	  			  2. Stores position and yaw orientation errors in state vectors;
	*				  3. Stores derivatives of position error and orientation error in state vectors;
	* 				  4. Stores second derivatives of desired position and orientation in state vectors;
	*                 5. Computes the FL control law;
	*                 6. Stabilishes a condition, with 'threshold', that defines a tolerance for position error;
	*                 7. Stabilishes a condition that defines a upper and lower limit for saturation each input;
	*                 8. Returns a 4x1 input vector;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector4d Drone::getFLControlLaw (void) {

		Vector4d xError, xIntError;
		Vector4d dx;
		Vector4d dxError;
		Vector4d d2xDesired;

		double deltaTAtual;

		xError.head(3) = positionError;
		xError(3) = yawError;

		dx.head(3) = dPosition;
		dx(3) = dYaw;

		dxError.head(3) = dPositionError;
		dxError(3) = dYawError;

		d2xDesired.head(3) = d2PositionDesired;
		d2xDesired(3) = d2YawDesired;

		u = d2xDesired  + Kp*xError + Kd*dxError;

		input = F1.inverse()*(u + F2*Rotation.transpose()*dx);
		
		// for(int i=0; i < 4;i++){
		// 	if(abs(xError(i)) < threshold(i)){
		// 		input(i) = 0.0;
		// 	}
		// 	if(abs(input(i)) > inputRange(i)){
		// 		if(input(i) > 0){
		// 			input(i) = inputRange(i);
		// 		} else {
		// 			input(i) = -inputRange(i);
		// 		}
		// 	}
		// }

		return input;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get PID Control Law
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Defines local variables and prints current position error, position desired and current postion;	
	* 	  			  2. Stores position and yaw orientation errors in state vectors;
	*				  3. Stores derivatives of position error and orientation error in state vectors;
	*                 4. Gets the current 'delta' time and computes the discrete integral of the position error (rectangle method) and stores it in "xIntError";
	*                 5. Prints the current position error, derivative position error and integral position error;
	*				  6. Computes the PID control law and prints its value;
	*                 7. Stabilishes a condition, with 'threshold', that defines a tolerance for position error;
	*                 8. Saturates each element of the input vector based on limits of the real actuator;
	*                 9. Returns a saturated 4x1 input vector;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector4d Drone::getPIDControlLaw (void) {

		Vector4d xError, xIntError;
		Vector4d dx;
		Vector4d dxError;

		double deltaTAtual;

		xError.head(3) = positionError;
		xError(3) = yawError;

		dx.head(3) = dPosition;
		dx(3) = dYaw;

		dxError.head(3) = dPositionError;
		dxError(3) = dYawError;

		deltaTAtual	  = getDeltaTimeNow();
		xIntError = getXIntError();
		xIntError = xIntError + xError*deltaTAtual;
		
		setXIntError(xIntError);

		u = Kp*xError + Kd*dxError + Ki*xIntError;


		// for(int i=0; i < 4;i++){

		// 	if(abs(xError(i)) < threshold(i)){
		// 		input(i) = 0.0;
		// 	}
		// 	else {
		// 		input(i) = u(i);
		// 	}

		// 	if(abs(input(i)) > inputRange(i)){
		// 		if(input(i) > 0){
		// 			input(i) = inputRange(i);
		// 		} else {
		// 			input(i) = -inputRange(i);
		// 		}
		// 	}
		// }

		return input;
	}


	 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get Recursive LQR Control Law
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides - dxDesired instead of dx on input.
	*
	*  	 Description: 1. Stores current position and orientation (yaw) error, derivative position and orientation, second derivative position and orientation desireds;
	*				  2. Stores "xError" and "dxError" in "x";
	* 				  3. Initiates and Defines the covariance state nois matrix and the covariance noise observer matrix;
	*                 4. Initiates the "L" matrix from RLQR;
	*				  5. Initiates the RLQR gain;
	* 				  6. Defines the augmentad state space and stores in "Acont" and "Bcont";
	*				  7. Prints "Acont" and "Bcont";
	*				  8. Converts the continuous model ("Acont" and "Bcont") to a discrete model ans stores in "Adisc" and "Bdisc";
	*				  9. Prints "Adisc" and "Bdisc";
	*				  10. Calls the RLQR() function with the parameters obtained as arguments and updates "L", "Krlqr" and "P";
	*				  11. Computes the RLQR control law and prints it;
	*				  12. Computes the 4x1 input vector based on the formulation obtained from state space error method;
	*				  13. Prints the input vector;
	*                 14. Stabilishes a condition, with 'threshold', that defines a tolerance for position error;
	*                 15. Saturates each element of the input vector based on limits of the real actuator;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector4d Drone::getRecursiveLQRControlLaw (void) {

			Vector4d xError,dxError,dx, dxDesired, d2xDesired;
			Vector8d x;

			xError.head(3) = positionError;
			xError(3) = yawError;

			dx.head(3) = dPosition;
			dx(3) = dYaw;

			dxError.head(3) = dPositionError;
			dxError(3) = dYawError;

			dxDesired.head(3) = dPositionDesired;
			dxDesired(3) 	  = dYawDesired;

			d2xDesired.head(3) = d2PositionDesired;
			d2xDesired(3) = d2YawDesired;

			x << dxError, xError;

			Matrix4d Rr;
			Matrix8d Acont,Adisc,Qr,L;
			Matrix8x4 Bcont,Bdisc;


			Rr << 1.0, 0.0, 0.0, 0.0,
				  0.0, 1.0, 0.0, 0.0,
				  0.0, 0.0, 1.0, 0.0,
				  0.0, 0.0, 0.0, 1.0;

			Rr = 0.15*Rr;

			Qr << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
				  0.0, 1.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
				  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  0.0, 0.0,
				  0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  0.0, 0.0,
				  0.0, 0.0, 0.0, 0.0, 1.0, 0.0,  0.0, 0.0,
				  0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  0.0, 0.0,
				  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0,
				  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 5.0;

			Qr = 1e-1*Qr;

			Klqr = Klqr.Zero();

			Acont << -F2*Rotation.transpose(), MatrixXd::Zero(4,4),
					 MatrixXd::Identity(4,4),  MatrixXd::Zero(4,4);

			Bcont << MatrixXd::Identity(4,4),
					 MatrixXd::Zero(4,4);

			Conversion::c2d(Adisc,Bdisc,Acont,Bcont,0.02);

			RecursiveLQR(Klqr,Adisc,Bdisc,Rr,Qr);
		
			u = Klqr*x; 
			
			input = F1.inverse()*(u + d2xDesired + F2*Rotation.transpose()*dxDesired);

			// for(int i=0; i < 4;i++){

			// 	if(abs(xError(i)) < threshold(i)){
			// 		input(i) = 0.0;
			// 	}

			// 	if(abs(input(i)) > inputRange(i)){
			// 		if(input(i) > 0){
			// 			input(i) = inputRange(i);
			// 		} else {
			// 			input(i) = -inputRange(i);
			// 		}
			// 	}
			// }

			return input;
		}

	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get Robust Control Law
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides - dxDesired instead of dx on input.
	*
	*  	 Description: 1. Stores current position and orientation (yaw) error, derivative position and orientation, second derivative position and orientation desireds;
	*				  2. Stores "xError" and "dxError" in "x";
	* 				  3. Initiates and Defines the covariance state nois matrix and the covariance noise observer matrix;
	*                 4. Initiates the "L" matrix from RLQR;
	*				  5. Initiates the RLQR gain;
	* 				  6. Defines the augmentad state space and stores in "Acont" and "Bcont";
	*				  7. Prints "Acont" and "Bcont";
	*				  8. Converts the continuous model ("Acont" and "Bcont") to a discrete model ans stores in "Adisc" and "Bdisc";
	*				  9. Prints "Adisc" and "Bdisc";
	*				  10. Calls the RLQR() function with the parameters obtained as arguments and updates "L", "Krlqr" and "P";
	*				  11. Computes the RLQR control law and prints it;
	*				  12. Computes the 4x1 input vector based on the formulation obtained from state space error method;
	*				  13. Prints the input vector;
	*                 14. Stabilishes a condition, with 'threshold', that defines a tolerance for position error;
	*                 15. Saturates each element of the input vector based on limits of the real actuator;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector4d Drone::getRobustControlLaw (void) {

			Vector4d xError,dxError,dx, dxDesired, d2xDesired,u;
			Vector8d x;

			xError.head(3) = positionError;
			xError(3) = yawError;

			dx.head(3) = dPosition;
			dx(3) = dYaw;

			dxError.head(3) = dPositionError;
			dxError(3) = dYawError;

			dxDesired.head(3) = dPositionDesired;
			dxDesired(3) 	  = dYawDesired;

			d2xDesired.head(3) = d2PositionDesired;
			d2xDesired(3) = d2YawDesired;

			x << dxError, xError;

			Matrix4d Rr;
			Matrix8d Acont,Adisc,Qr,L;
			Matrix8x4 Bcont,Bdisc;
			Matrix4x8 Krlqr;

			Rr = Rr.Identity();
			Rr = 0.15*Rr;
			
			
			Qr = Qr.Identity();
			Qr.block<2,2>(6,6) << 10.0, 0,
								   0, 5.0;
			Qr = 0.1*Qr;

			L = L.Zero();
			Krlqr = Krlqr.Zero();

			Acont << -F2*Rotation.transpose(), MatrixXd::Zero(4,4),
					 MatrixXd::Identity(4,4),  MatrixXd::Zero(4,4);

			Bcont << MatrixXd::Identity(4,4),
					 MatrixXd::Zero(4,4);

			Conversion::c2d(Adisc,Bdisc,Acont,Bcont,0.01);

			RLQR(L,Krlqr,Adisc,Bdisc,Rr,Qr);

			u = Krlqr*x;

			input = (u + d2xDesired + F2*Rotation.transpose()*dxDesired);

			input = F1.inverse()*input;

			// for(int i=0; i < 4;i++){

			// 	if(abs(xError(i)) < threshold(i)){
			// 		input(i) = 0.0;
			// 	}

			// 	if(abs(input(i)) > inputRange(i)){
			// 		if(input(i) > 0){
			// 			input(i) = inputRange(i);
			// 		} else {
			// 			input(i) = -inputRange(i);
			// 		}
			// 	}
			// }			
			return input;
		}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: get LQR Control Law
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Stores current position and orientation (yaw) error, derivative position and orientation, second derivative position and orientation desireds;
	*				  2. Stores "xError" and "dxError" in "x";
	* 				  3. Initiates and Defines the covariance state nois matrix and the covariance noise observer matrix;
	*                 4. Initiates the "L" matrix from RLQR;
	*				  5. Initiates the RLQR gain;
	* 				  6. Defines the augmentad state space and stores in "Acont" and "Bcont";
	*				  7. Prints "Acont" and "Bcont";
	*				  8. Converts the continuous model ("Acont" and "Bcont") to a discrete model ans stores in "Adisc" and "Bdisc";
	*				  9. Prints "Adisc" and "Bdisc";
	*				  10. Calls the RLQR() function with the parameters obtained as arguments and updates "L", "Krlqr" and "P";
	*				  11. Computes the RLQR control law and prints it;
	*				  12. Computes the 4x1 input vector based on the formulation obtained from state space error method;
	*				  13. Prints the input vector;
	*                 14. Stabilishes a condition, with 'threshold', that defines a tolerance for position error;
	*                 15. Saturates each element of the input vector based on limits of the real actuator;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector4d Drone::getLQRControlLaw (void) {

			Vector4d xError,dxError,dx, dxDesired, d2xDesired,u;
			Vector8d x;

			xError.head(3) = positionError;
			xError(3) = yawError;

			dx.head(3) = dPosition;
			dx(3) = dYaw;

			dxError.head(3) = dPositionError;
			dxError(3) = dYawError;

			dxDesired.head(3) = dPositionDesired;
			dxDesired(3) 	  = dYawDesired;

			d2xDesired.head(3) = d2PositionDesired;
			d2xDesired(3) = d2YawDesired;

			x << dxError, xError;

			Matrix4d Rr;
			Matrix8d Acont,Adisc,Qr;
			Matrix8x4 Bcont,Bdisc;
			Matrix4x8 Klqr;

			Rr = 0.0001*Rr.Identity();
			Qr = 0.05*Qr.Identity();

			Klqr <<   19.2462,         0,        0,         0,   18.4566,         0,         0,         0,
        					0,   19.2653,        0,         0,         0,   18.4532,         0,         0,
        					0,         0,  16.8239,         0,         0,         0,   18.9011,         0,
        					0,         0,        0,   17.4210,         0,         0,         0,   18.7914;

			Klqr << 0.1*Klqr;			        					

			u = -Klqr*x;

			input = F1.inverse()*(u + d2xDesired + F2*Rotation.transpose()*dxDesired);

			// for(int i=0; i < 4;i++){

			// 	if(abs(xError(i)) < threshold(i)){
			// 		input(i) = 0.0;
			// 	}

			// 	if(abs(input(i)) > inputRange(i)){
			// 		if(input(i) > 0){
			// 			input(i) = inputRange(i);
			// 		} else {
			// 			input(i) = -inputRange(i);
			// 		}
			// 	}
			// }			
			return input;
		}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #####################################            REGULAR FUNCTIONS                 ########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: initDroneParam
	*	  Created by: jrsbenevides
	*  Last Modified: Sep 16th 2019
	*
	*  	 Description: 1. Initialize essential functions for drone operation;
	*				  2. Initialize parameters and default values;		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::initDroneParam(void){

		isOdomStarted 			= false;
		isControllerStarted 	= true;
		isEKFonline				= false;
		isFirstEKFit			= true;
		tokenEKF				= true;

		position 				= position.Zero();
		position0 				= position0.Zero();
		positionDesired 		= positionDesired.Zero();
		positionError 			= positionError.Zero();
		dPosition 				= dPosition.Zero();
		dPositionDesired 		= dPositionDesired.Zero();
		dPositionError 			= dPositionError.Zero();
		d2PositionDesired 		= d2PositionDesired.Zero();
		dRPY 					= dRPY.Zero();
		orientation 			<< 1, 0, 0, 0;
		linearVel 				= linearVel.Zero();
		angularVel 				= angularVel.Zero();
		rpy 					= rpy.Zero();
		roll 					= 0;
		pitch 					= 0;
		yaw 					= 0;
		yaw0					= 0;
		yawDesired 				= 0;
		yawError 				= 0;
		
		timeNow 				= 0;
		timePast 				= 0;
		deltaTime				= 0;
		tStart					= 0;

		dYaw 					= 0;
		dYawDesired 			= 0;
		dYawError 				= 0;	
		d2YawDesired 			= 0 ;

		RotGlobal 				= RotGlobal.Identity();
		Rotation 				= Rotation.Identity();

		//K = K.Zero();
		K 						<<  1.74199, 0.94016, 1.54413, 0.89628, 3.34885, 3.29467, 6.51209, 3.92187;
		Kstart 					<<  1.00000, 1.00000, 1.00000, 1.00000, 1.00000, 1.00000, 1.00000, 1.00000;
		F1 						= F1.Identity();
		F2 						= F2.Identity();

		Kp 						= Kp.Zero();
		Kd 						= Kd.Zero();
		Ki 						= Ki.Zero();

		P 						= P.Identity();

		u 						= u.Zero();
		threshold 				= threshold.Zero();
		input 					= input.Zero();
		inputSat 				= inputSat.Zero();
		inputRange 				= inputRange.Zero();

		xIntError 				= xIntError.Zero();

		A_kalman				= A_kalman.Identity();
		P_kalman				= P_kalman.Identity();
		PAng_kalman				= PAng_kalman.Identity();
		Q_kalman				= 15*Q_kalman.Identity();
		
		Q_kalman.block<3,3>(0,0) <<  1,  0,  0,
									 0,  1,  0,
									 0,  0,  1;

		R_kalman				= 0.1*R_kalman.Identity();

		H_kalman				<< H_kalman.Zero();

		H_kalman.block<3,3>(0,0) << 1, 0, 0, 
								    0, 1, 0, 
								    0, 0, 1;

		x_kalman				= x_kalman.Zero();
		xAng_kalman				= xAng_kalman.Zero();	

		x_EKF					= x_EKF.Zero();
		setXfromEKF(x_EKF);

		F_EKF					= F_EKF.Identity();
		Q_EKF					= Q_EKF.Identity();
		R_EKF					= R_EKF.Identity();
		P_EKF					= P_EKF.Identity();

		deltaM           		= deltaM.Zero();
		deltaN           		= deltaN.Zero();

		updateRateEKF 			= 10;	

		Klqr 					= Klqr.Zero();
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: DvKalman
	*	  Created by: jrsbenevides
	*  Last Modified: March 7th 2018 
	*
	*  	 Description: Estimates linear velocity from Vicon position - Kalman Filter.
	*				  1. Updates A matrix based on current sampling time;
	* 				  2. Reads variable and error covariance estimate from previous time instant;
	*				  3. Performs simple Kalman Filter;	
	* 				  4. Stores variable and error covariance estimate from this time instant;		  
	* 				  5. Returns estimated velocity.		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector3axes Drone::DvKalman(const Vector3axes& posCurrent, const double& timeNow, const double& timePast){
		
		double dt;
		Vector3axes estLinearVel, z;

		Matrix3d 	aux;
		Matrix9d 	P, Pp;
		Vector9d 	x, xp;
		Matrix9x3	K;

		z = posCurrent;

		dt 		= timeNow - timePast;

		A_kalman.block<3,3>(0,3) << dt,  0,  0,
									 0, dt,  0,
									 0,  0, dt;

		A_kalman.block<3,3>(3,6) << dt,  0,  0,
									 0, dt,  0,
									 0,  0, dt;									 
		
		x 	= getKalmanX(); 
		P 	= getKalmanP();

		xp 	= A_kalman*x;
		Pp 	= A_kalman*P*A_kalman.transpose()  + Q_kalman;
		aux = H_kalman*Pp*H_kalman.transpose() + R_kalman;
		K 	= Pp*H_kalman.transpose()*aux.inverse();
		x 	= xp + K*(z - H_kalman*xp);
		P 	= Pp - K*H_kalman*Pp;

		setKalmanP(P);
		setKalmanX(x);

		estLinearVel 	<<  x.segment<3>(3); //vector.segment<n>(i) =  n elements starting at position i
		
		return estLinearVel;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: DwKalman
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: Estimates angular velocity from Vicon position - Kalman Filter.
	*				  1. Updates A matrix based on current sampling time;
	* 				  2. Reads variable and error covariance estimate from previous time instant;
	*				  3. Performs simple Kalman Filter;	
	* 				  4. Stores variable and error covariance estimate from this time instant;		  
	* 				  5. Returns estimated velocity.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Vector3axes Drone::DwKalman(const VectorQuat& orientCurrent, const double& timeNow, const double& timePast){

		double dt;
		Vector3axes estAngularVel, rpyCurrent, z;

		Matrix3d 	aux;
		Matrix9d 	P, Pp;
		Vector9d 	x, xp;
		Matrix9x3	K;

		Conversion::quat2angleZYX(rpyCurrent,orientCurrent);

		z = rpyCurrent;

		dt 		= timeNow - timePast;

		A_kalman.block<3,3>(0,3) << dt,  0,  0,
									 0, dt,  0,
									 0,  0, dt;

		A_kalman.block<3,3>(3,6) << dt,  0,  0,
									 0, dt,  0,
									 0,  0, dt;											 
		
		x 	= getKalmanXAngular(); 
		P 	= getKalmanPAngular();

		xp 	= A_kalman*x;
		Pp 	= A_kalman*P*A_kalman.transpose() + Q_kalman;
		aux = H_kalman*Pp*H_kalman.transpose() + R_kalman;
		K 	= Pp*H_kalman.transpose()*aux.inverse();
		x 	= xp + K*(z - H_kalman*xp);
		P 	= Pp - K*H_kalman*Pp;

		setKalmanPAngular(P);
		setKalmanXAngular(x);

		estAngularVel 	<<  x.segment<3>(3);
		
		return estAngularVel;
	}	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: inputSaturation
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: Defines a limitation for input signal (usually [-1,1])
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::inputSaturation(Vector4d& input){

		for(int i=0; i < 4;i++){

			if(abs(input(i)) > inputRange(i)){
				if(input(i) > 0){
					input(i) = inputRange(i);
				} else {
					input(i) = -inputRange(i);
				}
			}
		}
	}		

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: RecursiveLQR
	*	  Created by: Marlon
	*  Last Modified: jrsbenevides - Removed L from function
	*
	*  	 Description: 1. Receives the RecursiveLQR standard parameter matrices;
	*				  2. Defines and computes other parameters of the regulator;
	* 				  3. Computes the RLQR gain;
	*                 4. Updates the covariance matrix "P";
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::RecursiveLQR(Matrix4x8& Klqr, const Matrix8d& F, const Matrix8x4& G, const Matrix4d& Rr, const Matrix8d& Qr){
		

		Matrix4d Rrinv = Rr.inverse(); //For some reason I could not make Rr.inverse() directly into M.
		Matrix8d Finc;
		Matrix8x4 Ginc;
		Matrix8d sigma;
		Matrix8d Iest;
		Matrix8d Fest;
		Matrix8x4 Gest;
		MatrixSizeV2 V;
		MatrixSizeM2 M;
		MatrixSizeU2 U;
		MatrixSizeMcal Mcal;


		const int n = 8; //n of rows - F
		const int m = 4; //n of columns - G
		const int l = 4; //n of rows - Ef
		const int t = n;


		// PARA O RLQ RECURSIVO:
		sigma = sigma.Zero();


		Finc = F;
		Ginc = G;

		Fest << F;
		Gest << G;
		Iest << MatrixXd::Identity(n,n);

		V << MatrixXd::Zero(n+m,2*n+m),
			 MatrixXd::Zero(n,n+m), -MatrixXd::Identity(n,n),
			 MatrixXd::Zero(t,n+m), Fest,
			 MatrixXd::Identity(n+m,n+m),MatrixXd::Zero(n+m,n);

		U << MatrixXd::Zero(12,8),
			 -MatrixXd::Identity(8,8),
			 Fest,
			 MatrixXd::Zero(12,8);


		M << P.inverse(), 		   	  MatrixXd::Zero(n,n+m+t),						MatrixXd::Identity(n,n),		MatrixXd::Zero(n,m),
			 MatrixXd::Zero(m,n),  	  Rrinv,			                			MatrixXd::Zero(m,2*n+t),		MatrixXd::Identity(m,m),
			 MatrixXd::Zero(n,n+m),							   Qr.inverse(),		MatrixXd::Zero(n,n+m+t),
			 MatrixXd::Zero(t,2*n+m),						 						sigma, 		   	   Iest,	-Gest,
			 MatrixXd::Identity(n,n), MatrixXd::Zero(n,n+m), 					    Iest.transpose(),  MatrixXd::Zero(n,n+m),
			 MatrixXd::Zero(m,n), 	  MatrixXd::Identity(m,m), MatrixXd::Zero(m,n), -Gest.transpose(), MatrixXd::Zero(m,n+m);


		Mcal = V.transpose()*M.inverse()*U;

		Klqr  = Mcal.block<m,n>(n,0);
		P 	  = Mcal.block<n,n>(n+m,0);

	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: RLQR
	*	  Created by: jrsbenevides and dalsochio
	*  Last Modified:
	*
	*  	 Description: 1. Receives the RLQR standard parameter matrices;
	*				  2. Defines and computes other parameters of the regulator;
	* 				  3. Computes the RLQR gain;
	*                 4. Updates the covariance matrix "P";
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Drone::RLQR(Matrix8d& L, Matrix4x8& Krlqr, const Matrix8d& F, const Matrix8x4& G, const Matrix4d& Rr, const Matrix8d& Qr){

        double alpha;
        double beta;
        double gamma;

        Matrix4x8 Ef;
        Matrix4d Eg, Efaux0, Efaux1, auxLambda;
        Matrix4d Rrinv = Rr.inverse(); 

        Matrix8d Finc;
        Matrix8x4 Ginc;
        Matrix8x4 H;
        
        Matrix12x12 sigma;
        Matrix12x8 Iest;
        Matrix12x8 Fest;
        Matrix12x4 Gest;
        
        MatrixSizeV1 V;
        MatrixSizeM1 M;
        MatrixSizeU1 U;
        MatrixSizeMcal Mcal;

        double lambda;
        double delta = 1*randwithin(-1,1);

        const int n = 8; //# of rows - F
        const int m = 4; //# of columns - G
        const int l = 4; //# of rows - Ef
        const int t = n+4;

        long double mi = 1e15;


        H  << 1, 1, 0, 0,
              1, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1,
              MatrixXd::Zero(4,4);

        alpha = 0.295;
        beta  = 0.125;
        gamma = 0.285;

        Efaux0 << Qr.block<4,4>(0,0);
        Efaux1 << Qr.block<4,4>(4,4);

        Ef << alpha*Efaux0, gamma*Efaux1;
        Eg << beta*Rr;

        auxLambda = mi*H.transpose()*H;
        lambda = auxLambda.norm() + 0.01;

        sigma << (1/mi)*MatrixXd::Identity(8,8)-(1/lambda)*H*H.transpose(), MatrixXd::Zero(8,4),
                 MatrixXd::Zero(4,8),                         (1/lambda)*MatrixXd::Identity(4,4);

        Finc = F + delta*H*Ef;
        Ginc = G + delta*H*Eg;

        Fest << F, Ef;
        Gest << G, Eg;
        Iest << MatrixXd::Identity(n,n),
                MatrixXd::Zero(l,n);

        V << MatrixXd::Zero(n+m,2*n+m),
             MatrixXd::Zero(n,n+m), -MatrixXd::Identity(n,n),
             MatrixXd::Zero(t,n+m), Fest,
             MatrixXd::Identity(n+m,n+m),MatrixXd::Zero(n+m,n);

        U << MatrixXd::Zero(12,8),
             -MatrixXd::Identity(8,8),
             Fest,
             MatrixXd::Zero(12,8);


        M << P.inverse(),                  MatrixXd::Zero(n,n+m+t),                        MatrixXd::Identity(n,n),        MatrixXd::Zero(n,m),
        	 MatrixXd::Zero(m,n),  	  Rrinv,			                			MatrixXd::Zero(m,2*n+t),		MatrixXd::Identity(m,m),
			 MatrixXd::Zero(n,n+m),							   Qr.inverse(),		MatrixXd::Zero(n,n+m+t),
			 MatrixXd::Zero(t,2*n+m),						 						sigma, 		   	   Iest,	-Gest,
			 MatrixXd::Identity(n,n), MatrixXd::Zero(n,n+m), 					    Iest.transpose(),  MatrixXd::Zero(n,n+m),
			 MatrixXd::Zero(m,n), 	  MatrixXd::Identity(m,m), MatrixXd::Zero(m,n), -Gest.transpose(), MatrixXd::Zero(m,n+m);		


		Mcal = V.transpose()*M.inverse()*U;

		L 	  = Mcal.block<n,n>(0,0);
		Krlqr = Mcal.block<m,n>(n,0);
		P 	  = Mcal.block<n,n>(n+m,0);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: rand within
	*	  Created by: jrsbenevides
	*  Last Modified:
	*  	 Description: Computes a random value between two numbers based on a uniform distribution.
	*  	 Description: 1. Receives two parameters (upper and bottom limits) for updates the contraction matrix within RLQR;
	*				  2. Computes difference between upper and bottom limits (double) and stores as "range";
	* 				  3. Returns a result of an operation that involves "floor", "range" and a random number (rand funcion);
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double Drone::randwithin(double floor,double ceiling){
		double range = (ceiling - floor);

		return (floor + (range * rand()));
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: norm rnd
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: Function to be used in future versions only;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double Drone::normrnd(double mean, double stdDev) {
			    double u, v, s;
			    do {
			        u = ((double)rand()) * 2.0 - 1.0;
			        v = ((double)rand()) * 2.0 - 1.0;
			        s = u * u + v * v;
			    } while (s >= 1 || s == 0);
			    double mul = sqrt(-2.0 * log(s) / s);
			    return mean + stdDev * u * mul;
	}
} // namespace DRONE
