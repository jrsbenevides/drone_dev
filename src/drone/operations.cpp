/*
 * operations.cpp
 *
 *  Created on: 10/07/2017
 *      Author: roberto
 */


#include "drone/operations.h"

namespace DRONE {



//	VectorQuat Conversion::angle2quatZYX(const double& yaw, const double& pitch, const double& roll){
//		VectorQuat q; // q = [w, x, y, z]'
//		q(0) = cos(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);
//		q(1) = cos(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll) - sin(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll);
//		q(2) = cos(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll);
//		q(3) = sin(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) - cos(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);
//
//		return q;
//	}

	void Conversion::angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll){
		// q = [w, x, y, z]'
		// Avaliar depois se vale a pena otimizar...
		//Verificado em 16/02/18
		q(0) = cos(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);
		q(1) = cos(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll) - sin(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll);
		q(2) = cos(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll);
		q(3) = sin(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) - cos(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);	
	}

	void Conversion::quat2angleZYX(Vector3axes& Angles,const VectorQuat& q){

		// Angles(0) = atan2( 2*(q(2)*q(3) + q(0)*q(1)) , (q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3)) ); // Roll angle - axis X
		// Angles(1) = asin( -2*(q(1)*q(3) -q(0)*q(2)) );                                                    // Pitch angle - axis Y
		// Angles(2) = atan2( 2*(q(1)*q(2) + q(0)*q(3)) , q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3) );   // Yaw angle - axis Z

		//Verified on Feb/06/2018 - Joao and Marlon
		Angles(0) = atan2( 2*(q(2)*q(3) + q(0)*q(1)) , 1 - 2*(q(1)*q(1) + q(2)*q(2)) ); 	// Roll angle - axis X
		Angles(1) = asin(  2*(q(0)*q(2) - q(1)*q(3)) );                                   	// Pitch angle - axis Y
		Angles(2) = atan2( 2*(q(1)*q(2) + q(0)*q(3)) , 1 - 2*(q(2)*q(2) + q(3)*q(3)) );   	// Yaw angle - axis Z

	}


	void Conversion::quat2dcmZYX(Matrix3d& R, const VectorQuat& q) {

		R << q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)+q(0)*q(3)),             		   2*(q(1)*q(3)-q(0)*q(2)),
			 2*(q(1)*q(2)-q(0)*q(3)),         	      q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3),     2*(q(2)*q(3)+q(0)*q(1)),
			 2*(q(1)*q(3)+q(0)*q(2)),                 2*(q(2)*q(3)-q(0)*q(1)),                     q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);

	}


	void Conversion::quatmultiply(VectorQuat& s, const VectorQuat& p, const VectorQuat& q){

		Matrix4d P;

		P << p(0), -p(1), -p(2), -p(3),
			 p(1),  p(0), -p(3),  p(2),
			 p(2),  p(3),  p(0), -p(1),
			 p(3), -p(2),  p(1),  p(0);

		s = P*q;
	}

	void Conversion::quatconj(VectorQuat& s, const VectorQuat& q){

		s << q(0),-q(1),-q(2), -q(3);

	}


	void Conversion::c2d(Matrix8d& Adisc,Matrix8x4& Bdisc,const Matrix8d& Acont,const Matrix8x4& Bcont, double tS){

		/*Computes Ad through Taylor Series' expansion of e^(AT) and Bd when A in non singular*/

		Adisc = MatrixXd::Identity(8,8);
		Matrix8d Bdaux = MatrixXd::Identity(8,8);
		Matrix8d aux = MatrixXd::Identity(8,8);
		double inv;

		for(int i=1;i<=30;i++)
		{
		   inv = (1/(double)i);
		   aux *= inv*tS*Acont;
		   Adisc = Adisc + aux;
		   Bdaux = Bdaux + (1/(double)(i+1))*aux;
		}

		Bdisc =  Bdaux*Bcont*tS;
	}

	void Conversion::geodetic2ecef(Vector3d& pe, const Vector3d& llh){

		double lat = llh(0);
		double lon = llh(1);
		double h = llh(2);

		// World Geodetic System 1984
		// b = 6356752.31424518; SemiminorAxis [m]
		double a = 6378137; // SemimajorAxis [m]
		double e = 0.0818191908426215; // Eccentricity
		double e2 = e*e;

		double RN = a/(pow((1-e2*pow(sin(lat),2)),0.5));

		pe(0) = (RN + h)*cos(lat)*cos(lon);    // x
		pe(1) = (RN + h)*cos(lat)*sin(lon);    // y
		pe(2) = (RN*(1-e2)+h)*sin(lat);        // z
	}

	void Conversion::ecef2ned(Vector3d& pt, const Vector3d& pe, const Vector3d& llh0){
		double lat0 = llh0(0);
		double lon0 = llh0(1);
		double h0 = llh0(2);

		Matrix3d RTE;
		RTE << -sin(lat0)*cos(lon0), -sin(lat0)*sin(lon0),   cos(lat0),
			   -sin(lon0),            cos(lon0),   0,
			   -cos(lat0)*cos(lon0), -cos(lat0)*sin(lon0),  -sin(lat0);

		Vector3d pe0;

		geodetic2ecef(pe0,llh0);

		pt = RTE*(pe - pe0);
	}


} // namespace DRONE



