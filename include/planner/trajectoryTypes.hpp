#ifndef TRAJECTORYTYPES_HPP_
#define TRAJECTORYTYPES_HPP_

#include "drone/definitions.h"
#include <string>

namespace DRONE
{

enum Trajectory
{
	eightShape = 0U,
	circleXY,
	ident,
	circleZXY,
	straightLine,
	wayPoint,
	followTruck
};

class TrajectoryParameters
{
public:
	
	float32	angularFrequency;
	float32	amplitude;
	float32 averageLinearSpeed;
	
	VectorFive 	poseDesired;
	
	Vector3axes cTx;
	Vector3axes cTy;
	Vector3axes cTz;
	Vector3axes cTyaw;
	
	Trajectory trajectory;

	/*
	* @brief Based on the chosen trajectory, this will update the desired angular frequency.
	*/
	float32 updateAngularFrequency()
	{
		if (trajectory & eightShape)
		{
			angularFrequency = 2 * M_PI * averageLinearSpeed / (6.097F * amplitude);
		}
		else
		{
			angularFrequency = averageLinearSpeed / amplitude;
		}
	}

	/*
	* @brief Loads the auxiliary trajectory coefficients. Needed for specific trajectories only.
	*/
	void setTrajectoryCoefficients(void) 
	{

		float32 x = poseDesired(0);
		float32 y = poseDesired(1);
		float32 z = poseDesired(2);
		float32 yaw = poseDesired(3);
		float32 tf = poseDesired(4);
		float32 tf3 = tf * tf * tf;
		float32 tf4 = tf3 * tf;
		float32 tf5 = tf4 * tf;

		cTx(0) = 10.F * x / tf3;
		cTx(1) = -15.F * x / tf4;
		cTx(2) = 6.F * x / tf5;

		cTy(0) = 10.F * y / tf3;
		cTy(1) = -15.F * y / tf4;
		cTy(2) = 6.F * y / tf5;

		cTz(0) = 10.F * z / tf3;
		cTz(1) = -15.F * z / tf4;
		cTz(2) = 6.F * z / tf5;

		cTyaw(0) = 10.F * yaw / tf3;
		cTyaw(1) = -15.F * yaw / tf4;
		cTyaw(2) = 6.F * yaw / tf5;
	}

	/*
	* @brief Loads the trajectory input
	*/
	void setTrajectory(const string& trajectoryInput) {

		Trajectory defaultTrajectory = circleXY;

		if (isStringEqual(trajectoryInput,"eightShape"))
		{
			trajectory = eightShape;
		}
		else if (isStringEqual(trajectoryInput, "circleXY"))
		{
			trajectory = circleXY;
		}
		else if (isStringEqual(trajectoryInput, "ident"))
		{
			trajectory = ident;
		}
		else if (isStringEqual(trajectoryInput, "circleZXY"))
		{
			trajectory = circleZXY;
		}
		else if (isStringEqual(trajectoryInput, "straightLine"))
		{
			trajectory = straightLine;
		}
		else if (isStringEqual(trajectoryInput, "wayPoint"))
		{
			trajectory = wayPoint;
		}
		else if (isStringEqual(trajectoryInput, "followTruck"))
		{
			trajectory = followTruck;
		}
		else
		{
			trajectory = defaultTrajectory;
		}
	}

private:

	/*
	* @brief Compares the content of a string and a char pointer
	*/
	static bool isStringEqual(const string& stringInput, const uint8* word)
	{
		return (stringInput.compare(word) == 0);
	}
}; // TrajectoryParameters

} // namespace DRONE
#endif // TRAJECTORYTYPES_HPP_