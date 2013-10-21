/*
 * Wall.h
 *
 *  Created on: Oct 17, 2013
 *      Author: robo
 */

#ifndef WALL_H_
#define WALL_H_
#include "Vec2D.h"
#include <geometry_msgs/Point.h>

class Wall {
public:
	Wall(Vec2D<double> p1, Vec2D<double> p2, double w);
	double getDistance(const Vec2D<double> &p) const;
	double getDistance(const Vec2D<double> &p, const Vec2D<double> &dir) const;
	double getThickness() const;
	geometry_msgs::Point getPoint(double p) const;
	virtual ~Wall();
private:
	Vec2D<double> m_center;
	Vec2D<double> m_normal;
	double m_length_2;
	double m_width_2;
};

#endif /* WALL_H_ */
