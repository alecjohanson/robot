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

void PlaneEstimator::addPoint(Eigen::Matrix<int16_t,3,1> &m) {
	xx+=(int64_t)m[0]*(int64_t)m[0];
	xy+=(int64_t)m[0]*(int64_t)m[1];
	xz+=(int64_t)m[0]*(int64_t)m[2];
	yy+=(int64_t)m[1]*(int64_t)m[1];
	yz+=(int64_t)m[1]*(int64_t)m[2];
	_x+=m[0];
	_y+=m[1];
	_z+=m[2];
	zz+=(int64_t)m[2]*(int64_t)m[2];
	numPoints++;
}

Eigen::Vector3d &PlaneEstimator::getNormal() {
	if(isnan(dist)) calcPlane();
	return normal;
	//return cam2robot.row(2);
}

double PlaneEstimator::getDistance() {
	if(isnan(dist)) calcPlane();
	return dist;
}

size_t PlaneEstimator::getNumPoints() const {
	return numPoints;
}

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
