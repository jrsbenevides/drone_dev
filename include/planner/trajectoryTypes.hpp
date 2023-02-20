
#ifndef TRAJECTORYTYPES_HPP_
#define TRAJECTORYTYPES_HPP_

namespace DRONE
{

static const double PI = 3.14159265359F;

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
	
	double	angularFrequency; //wAng
	double	amplitude;
	double 	averageLinearSpeed;
	
	VectorFive 	poseDesired;
	
	Vector3axes cTx;
	Vector3axes cTy;
	Vector3axes cTz;
	Vector3axes cTyaw;
	
	Trajectory trajectory;

	/*
	* @brief Based on the chosen trajectory, this will update wAng, which should be the desired angular frequency.
	*/
	inline double updateAngularFrequency()
	{
		if (trajectory & eightShape)
		{
			angularFrequency = 2 * PI * averageLinearSpeed / (6.097 * amplitude);
		}
		else
		{
			angularFrequency = averageLinearSpeed / amplitude;
		}
	}

	/*
	* @brief Loads the auxiliary trajectory coefficients. Needed for specific trajectories only.
	*/
	void setTrajectoryCoefficients(void) {
		double x, y, z, yaw, tf, tf3, tf4, tf5;
		x = poseDesired(0);
		y = poseDesired(1);
		z = poseDesired(2);
		yaw = poseDesired(3);
		tf = poseDesired(4);
		tf3 = tf * tf * tf;
		tf4 = tf3 * tf;
		tf5 = tf4 * tf;

		cTx(0) = 10 * x / tf3;
		cTx(1) = -15 * x / tf4;
		cTx(2) = 6 * x / tf5;

		cTy(0) = 10 * y / tf3;
		cTy(1) = -15 * y / tf4;
		cTy(2) = 6 * y / tf5;

		cTz(0) = 10 * z / tf3;
		cTz(1) = -15 * z / tf4;
		cTz(2) = 6 * z / tf5;

		cTyaw(0) = 10 * yaw / tf3;
		cTyaw(1) = -15 * yaw / tf4;
		cTyaw(2) = 6 * yaw / tf5;
	}

	/*
	* @brief Loads the trajectory input
	*/
	void setTrajectory(const string& trajectoryInput) {

		Trajectory defaultTrajectory = circleXY;

		if (trajectoryInput.compare("eightShape") == 0)
		{
			trajectory = eightShape;
		}
		else if (trajectoryInput.compare("circleXY") == 0)
		{
			trajectory = circleXY;
		}
		else if (trajectoryInput.compare("ident") == 0)
		{
			trajectory = ident;
		}
		else if (trajectoryInput.compare("circleZXY") == 0)
		{
			trajectory = circleZXY;
		}
		else if (trajectoryInput.compare("straightLine") == 0)
		{
			trajectory = straightLine;
		}
		else if (trajectoryInput.compare("wayPoint") == 0)
		{
			trajectory = wayPoint;
		}
		else if (trajectoryInput.compare("followTruck") == 0)
		{
			trajectory = followTruck;
		}
		else
		{
			trajectory = defaultTrajectory;
		}
	}
}; // TrajectoryParameters

} // namespace DRONE
#endif // TRAJECTORYTYPES_HPP_