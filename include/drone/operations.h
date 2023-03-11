/*
 * operations.h
 *
 *  Created on: 10/07/2017
 *      Author: roberto
 */

#ifndef OPERATIONS_H_
#define OPERATIONS_H_


#include "ros/ros.h"
#include "definitions.h"
using namespace std;
using namespace DRONE::Types;

namespace DRONE 
{
namespace Conversion 
{
//VectorQuat angle2quatZYX(const double& yaw, const double& pitch, const double& roll);
void angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll);
//Vector3axes quat2angleZYX(VectorQuat q);
void quat2angleZYX(Vector3axes& Angles,const VectorQuat& q);
//Matrix3d quat2dcmZYX(VectorQuat q);
void quat2dcmZYX(Matrix3d& R, const VectorQuat& q);
//VectorQuat quatmultiply(VectorQuat q1, VectorQuat q2);
void quatmultiply(VectorQuat& s, const VectorQuat& p, const VectorQuat& q);
//VectorQuat quatconj(VectorQuat q);
void quatconj(VectorQuat& s, const VectorQuat& q);
//Vector3d geodetic2ecef(Vector3d llh);
void geodetic2ecef(Vector3d& pe, const Vector3d& llh);
//Vector3d ecef2ned(Vector3d pe, Vector3d llh0);
void ecef2ned(Vector3d& pt, const Vector3d& pe, const Vector3d& llh0);
void c2d(Matrix8d& Adisc, Matrix8x4& Bdisc, const Matrix8d& Acont, const Matrix8x4& Bcont, double tS);
	
} // namespace Conversion
} // namespace DRONE


#endif /* OPERATIONS_H_ */
