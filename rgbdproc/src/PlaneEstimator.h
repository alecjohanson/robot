/*
 * PlaneEstimator.h
 *
 *  Created on: Nov 7, 2013
 *      Author: robo
 */

#ifndef PLANEESTIMATOR_H_
#define PLANEESTIMATOR_H_

#include <Eigen/Dense>
#include <stdint.h>
#include <stddef.h>

class PlaneEstimator {
public:
	PlaneEstimator();
	~PlaneEstimator();
	void addPoint(Eigen::Matrix<int16_t,3,1> &m);
	Eigen::Vector3d &getNormal();
	double getDistance();
//	double getMSE();
	size_t getNumPoints() const;
private:
	void calcPlane(void);
	int64_t xx, xy, xz, yy, yz, zz;
	int64_t _x, _y, _z;
	size_t numPoints;
	Eigen::Vector3d normal;
	double dist;
	//double mse;
};

#endif /* PLANEESTIMATOR_H_ */
