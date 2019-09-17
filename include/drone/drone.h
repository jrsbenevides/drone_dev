/*
 * drone.h
 *
 *  Created on: 08/07/2017
 *      Author: roberto
 */

#ifndef DRONE_H_
#define DRONE_H_


#include "ros/ros.h"
#include "angles/angles.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"


#include "drone/definitions.h"
#include "drone/operations.h"

using namespace DRONE::Types;

namespace DRONE {

	class Drone {

		protected:

			int  isEKFonline;
			int  updateRateEKF;
			bool isOdomStarted;
			bool isControllerStarted;
			bool isViconStarted;
			bool isFirstEKFit;
			bool isDroneFlying;
			bool tokenEKF;
			
			Vector3axes position;
			Vector3axes position0;
			Vector3axes positionDesired;
			Vector3axes positionError;
			Vector3axes dPosition;
			Vector3axes dPositionDesired;
			Vector3axes dPositionError;
			Vector3axes d2PositionDesired;
			Vector3axes dRPY;
			Vector3axes linearVel;
			Vector3axes angularVel;
			Vector3axes rpy;
			
			Vector4d 	cmdvelMsg;

			VectorQuat 	orientation;
			

			double roll;
			double pitch;
			double yaw;
			double yaw0;
			double yawDesired;
			double yawError;
			double dYaw;
			double dYawDesired;
			double dYawError;
			double d2YawDesired;
			double pastTime;

			Matrix3d RotGlobal;
			
			Matrix4d Rotation;
			Matrix4d F1, F2;
			Matrix4d Kp, Kd, Ki;
			Matrix4x8 Klqr;
			
			Matrix9d 	A_kalman;
			Matrix9d 	P_kalman;
			Matrix9d 	PAng_kalman;
			Matrix9d 	Q_kalman;
			Matrix3x9	H_kalman;
			Matrix3d 	R_kalman;
			Vector9d	x_kalman;
			Vector9d	xAng_kalman;

			Vector32x1     x_EKF;
			Matrix4d	   R_EKF;
			Matrix4d	   deltaM, deltaN;
			MatrixSizeEKF  P_EKF, Q_EKF, F_EKF;

			Matrix8d P;

			Vector4d threshold;
			Vector4d inputRange;
			Vector4d u;
			Vector4d input;
			Vector4d inputSat;
			Vector4d xIntError;

			Vector8d K;
			Vector8d Kstart;

			double amplitude;
			double velMed;
			double timeNow;
			double timePast;
			double deltaTime;
			double timeOrigin;
			double tStart;

		public:

			Drone();
			void 		setPosition(const Vector3axes& positionValue);
			void 		setPosition0(const Vector3axes& positionValue, const double& yawValue);
			void 		setPositionDesired(const Vector3axes& positionValue);
			void 		setdPositionDesired(const Vector3axes& dPositionValue);
			void 		setd2PositionDesired(const Vector3axes& d2PositionValue);
			void 		setOrientation (const VectorQuat& orientationValue);
			void 		setLinearVel(const Vector3axes& linearVelValue);
			void 		setLinearVelVicon(const Vector3axes& linearVelValue);
			void 		setAngularVel(const Vector3axes& angularVelValue);
			void 		setK(const Vector8d& KValues);
			void 		setKstart(const Vector8d& KstartValues);
			void 		setYawDesired(const double& yawValue);
			void 		setdYawDesired(const double& dYawValue);
			void 		setd2YawDesired(const double& d2YawValue);
			void 		setKp(const Matrix4d& KpMatrix);
			void 		setKd(const Matrix4d& KdMatrix);
			void 		setKi(const Matrix4d& KiMatrix);
			void 		setThreshold(const Vector4d& upperLimit);
			void 		setInputRange(const Vector4d& inputRangeValue);
			void 		setIsOdomStarted(const bool& value);
			void 		setIsViconStarted(const bool& value);
			void 		setIsEKFonline(const int& value);
			void 		setUpdateRateEKF(const int& value);
			void 		setXIntError(const Vector4d& xIntErrorValue);
			void 		setTimeNow(const double& timeValue);
			void 		setTimeOrigin(const  double& timeValue);
			void 		setKalmanX(const Vector9d& value);
			void		setKalmanP(const Matrix9d& value);
			void 		setKalmanXAngular(const Vector9d& value);
			void		setKalmanPAngular(const Matrix9d& value);	
			void		setXfromEKF(const Vector32x1& value);
			void  		setCmdVel(const Vector4d& value);
			void 		setIsFlying(const bool& value);	
			void 		setEKFToken(const bool& value);		

			double 		getTimeNow(void);

			VectorQuat  getOrientation(void);
			
			double 		getRoll(void);
			double 		getPitch(void);
			double 		getYaw(void);
			double 		getDeltaTimeNow(void);
			double 		getTimeOrigin(void);
			double 		getThisTime(const double& thisTime);
			double 		randwithin(double floor,double ceiling);
			double 		normrnd(double mean, double stdDev);
			
			Matrix4d 	getF1(void);
			Matrix4d 	getF2(void);
			Matrix4d 	getKp(void);
			Matrix4d 	getKd(void);
			Matrix4d 	getKi(void);

			Vector3axes getPosition(void);
			Vector3axes getRPY(void);
			Vector3axes getLinearVelVicon(const Vector3axes& posCurrent, const Vector3axes& posPast, const double& timeNow, const double& timePast);
			Vector3axes getAngularVelVicon(const VectorQuat& orientCurrent, const VectorQuat& orientPast, const double& timeNow, const double& timePast);
			Vector3axes DvKalman(const Vector3axes& posCurrent, const double& timeNow, const double& timePast);		
			Vector3axes DwKalman(const VectorQuat& orientCurrent, const double& timeNow, const double& timePast);

			Vector4d 	getXIntError(void);
			Vector4d 	getHinfControlLaw (void); 		  // H-Infinite Controller
			Vector4d 	getFLControlLaw (void);   		  // Feedback Linearization Controller
			Vector4d 	getPIDControlLaw (void);		  // PID Controller
			Vector4d 	getRobustControlLaw (void); 	  // RLQR Controller
			Vector4d 	getLQRControlLaw (void);		  // LQR Controller
			Vector4d 	getRecursiveLQRControlLaw (void); // Recursive LQR
			Vector4d 	getThreshold(void);
			Vector4d 	getInputRange(void);
			Vector4d 	getCmdVel(void);		

			Vector8d 	getK(void);
			Vector8d 	getKstart(void);

			Vector9d 	getKalmanX(void);
			Vector9d 	getKalmanXAngular(void);
			Matrix9d 	getKalmanP(void);
			Matrix9d 	getKalmanPAngular(void);

			Vector32x1	getXfromEKF(void);
			
			int 		getIsEKFonline(void);
			int 		getUpdateRateEKF(void);

			void 		initDroneParam(void);
			void 		inputSaturation(Vector4d& input);
			void 		RLQR(Matrix8d& L, Matrix4x8& Krlqr, const Matrix8d& F, const Matrix8x4& G, const Matrix4d& Rr, const Matrix8d& Qr);
			void 		RecursiveLQR(Matrix4x8& Klqr, const Matrix8d& F, const Matrix8x4& G, const Matrix4d& Rr, const Matrix8d& Qr);
			
			bool 		getIsOdomStarted(void);
			bool 		getIsViconStarted(void);			
			bool 		getIsFlying(void);
			bool 		getEKFToken(void);
	};


}

#endif /* DRONE_H_ */
