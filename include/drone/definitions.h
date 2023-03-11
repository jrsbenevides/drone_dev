/*
 * definitions.h
 *
 *  Created on: 08/07/2017
 *      Author: roberto
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include "Eigen/Dense"
#include <math.h>
#include <cstdint>

using namespace std;
using namespace Eigen;

namespace DRONE 
{
// Constant variables
const float d2r = M_PI/180;
const float r2d = 180/M_PI;
const float T2G = 10e4;  	// Tesla to Gauss

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int8_t sint8;
typedef int16_t sint16;
typedef int32_t sint32;
typedef double float32;

namespace Types
{
// System sizes
const int size3axes = 3;
const int sizeFive = 5;
const int sizeQuat = 4;
const int size6d = 6;
const int size8d = 8;
const int sizeSigma = 9;
const int sizeDelta = 16;
const int sizeVrow = 20;
const int sizeEKF = 32;
const int sizeM = 41;
// const int sizeM 	= 44;
// New parameters - MARLON
const int sizeM1 = 44;
const int sizeM2 = 40;
const int size12d = 12;

// Eigen types
typedef Matrix<double, sizeFive, 1> 		 VectorFive;
typedef Matrix<double, sizeQuat, 1> 		 VectorQuat;
typedef Matrix<double, 1, sizeQuat>	 Vector1d4;
typedef Matrix<double, size3axes, 1> 		 Vector3axes;
typedef Matrix<double, size3axes, size6d>    Matrix3x6;
typedef Matrix<double, size3axes, sizeSigma> Matrix3x9;
typedef Matrix<double, size8d, 1>      	 Vector8d;
typedef Matrix<double, sizeSigma, 1>      	 Vector9d;
typedef Matrix<double, 1, size8d> 	 Vector1d8;
typedef Matrix<double, sizeDelta, 1> 	 	 Vector16x1;
typedef Matrix<double, sizeEKF, 1> 	 	 Vector32x1;
typedef Matrix<double, 1, sizeEKF> 	 	 Vector1x32;

typedef Matrix<double, sizeQuat, sizeQuat>  Matrix4d;
typedef Matrix<double, sizeQuat, size8d>  	 Matrix4x8;
typedef Matrix<double, size6d, size6d>    Matrix6d;
typedef Matrix<double, size6d, 1>    	 Vector6d;
typedef Matrix<double, size6d, size3axes> Matrix6x3;
typedef Matrix<double, size8d, size8d>    Matrix8d;
typedef Matrix<double, sizeSigma, sizeSigma> Matrix9d;
typedef Matrix<double, sizeSigma, size3axes> Matrix9x3;
typedef Matrix<double, size8d, sizeQuat>  Matrix8x4;
typedef Matrix<double, sizeSigma, size8d>    Matrix9d8;
typedef Matrix<double, sizeSigma, sizeQuat>  Matrix9d4;
typedef Matrix<double, sizeM, sizeVrow>  MatrixSizeV;
typedef Matrix<double, sizeM, sizeM>     MatrixSizeM;
typedef Matrix<double, sizeM, size8d>    MatrixSizeU;
typedef Matrix<double, sizeVrow, size8d>    MatrixSizeMcal;
typedef Matrix<double, sizeEKF, sizeEKF>   MatrixSizeEKF;
typedef Matrix<double, sizeEKF, sizeQuat>  Matrix32x4;
typedef Matrix<double, sizeQuat, sizeEKF>   Matrix4x32;

typedef Matrix<double, size12d, 1>         Vector12d;
typedef Matrix<double, size12d, sizeQuat>  Matrix12x4;
typedef Matrix<double, sizeQuat, size12d>   Matrix4x12;
typedef Matrix<double, size12d, size8d>    Matrix12x8;
typedef Matrix<double, size12d, size12d>   Matrix12x12;

typedef Matrix<double, sizeM1, sizeVrow>  MatrixSizeV1;
typedef Matrix<double, sizeM2, sizeVrow>  MatrixSizeV2;
typedef Matrix<double, sizeM1, sizeM1>    MatrixSizeM1;
typedef Matrix<double, sizeM2, sizeM2>    MatrixSizeM2;
typedef Matrix<double, sizeM1, size8d>    MatrixSizeU1;
typedef Matrix<double, sizeM2, size8d>    MatrixSizeU2;
//	typedef Matrix<double, size4values, 1> Vector4values;
//	typedef Matrix<double, sizeState, sizewNoise> MatrixG;
//	typedef Matrix<double, sizeObser, sizeState> MatrixH;
//	typedef Matrix<double, sizeState, sizeState> MatrixP;
//	typedef Matrix<double, sizewNoise, sizewNoise> MatrixQ;
//	typedef Matrix<double, sizeState, sizeState> MatrixQd;
//	typedef Matrix<double, sizevNoise, sizevNoise> MatrixR;
//	typedef Matrix<double, sizeState, sizeObser> MatrixK;
} // namespace Types

} // namespace DRONE


#endif /* DEFINITIONS_H_ */
