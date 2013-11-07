/*
 * PlaneEstimator.cpp
 *
 *  Created on: Nov 7, 2013
 *      Author: robo
 */

#include "PlaneEstimator.h"
#include <Eigen/LU>

PlaneEstimator::PlaneEstimator() :
	xx(0), xy(0), xz(0), yy(0), yz(0), zz(0),
	_x(0), _y(0), _z(0),
	numPoints(0),
	normal(NAN,NAN,NAN),
	dist(NAN)
{

}

PlaneEstimator::~PlaneEstimator() {
}

void PlaneEstimator::addPoint(int16_t x, int16_t y, int16_t z) {
	xx+=(int64_t)x*(int64_t)x;
	xy+=(int64_t)x*(int64_t)y;
	xz+=(int64_t)x*(int64_t)z;
	yy+=(int64_t)y*(int64_t)y;
	yz+=(int64_t)y*(int64_t)z;
	_x+=x;
	_y+=y;
	_z+=z;
	zz+=(int64_t)z*(int64_t)z;
	numPoints++;
}

Eigen::Vector3d &PlaneEstimator::getNormal() {
	if(isnan(dist)) calcPlane();
	return normal;
}

double PlaneEstimator::getDistance() {
	if(isnan(dist)) calcPlane();
	return dist;
}

size_t PlaneEstimator::getNumPoints() const {
	return numPoints;
}
/*
double PlaneEstimator::getMSE() {
	if(isnan(dist)) calcPlane();
	return mse;
}
*/

void PlaneEstimator::calcPlane(void) {
	Eigen::Matrix3d XtX;
	XtX<< xx, xy, xz,
		  xy, yy, yz,
		  xz, yz, zz;
	Eigen::Vector3d Xty;
	Xty<< -_x, -_y, -_z;
	normal=XtX.inverse()*Xty;
	//mse=(y-XtX*normal).norm();
	dist=1./normal.norm();
	normal*=dist;
}
