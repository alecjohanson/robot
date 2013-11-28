/*
 * Wall.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: robo
 *
 * This class represents a single straight wall.
 * Internally, it stores the center point, normal vector, thickness and length.
 * The wall can be queried for its shortest distance to a point
 * and for its distance to a point along a given ray. In all calculations,
 * the wall's nonzero thickness is taken into account.
 */

#include "Wall.h"
#include <cmath>
#include <cfloat>
#include <algorithm>

Wall::Wall(Vec2D<double> p1, Vec2D<double> p2, double w):
	m_center((p1+p2)*0.5),
	m_normal((p2-p1).rot90L().norm()),
	m_length_2((p2-p1).abs()*0.5),
	m_width_2(w*0.5)
{}

/*
 * Calculate the shortest distance between a point and any point on the wall.
 * \param p A point somewhere in the world plane.
 * \return The distance or 0.0 if the point is inside the wall.
 */
double Wall::getDistance(const Vec2D<double> &p) const {
	Vec2D<double> cp=p-m_center;
	double dnormal=cp*m_normal;
	double dcollinear=cp*m_normal.rot90L();
	double a=copysign(m_width_2,dnormal);
	double b=copysign(m_length_2,dcollinear);
	dnormal=fabs(dnormal);
	dcollinear=fabs(dcollinear);
	if(dnormal<m_width_2) {
		if(dcollinear<m_length_2) {
			//p is inside the wall
			return 0.0;
		} else {
			//p is closest to a short surface of the wall
			return dcollinear-m_length_2;
		}
	} else {
		if(dcollinear<m_length_2) {
			//p is closest to a long surface of the wall
			return dnormal-m_width_2;
		} else {
			//p is closest to a corner point of the wall.
			//We already know which one from the a,b factors.
			return (cp-m_normal*a-m_normal.rot90L()*b).abs();
		}
	}
}

typedef struct {
	double t,u;
} lineintersect_t;

/**
 * Calculate the intersection of two lines.
 * \param pt A point on the first line.
 * \param dt Direction of the first line.
 * \param pu A point on the second line.
 * \param du Direction of the second line.
 * \return Two scalars t,u so that the intersection point is pt+t*dt=pu+u*du.
 *   u=v=INFINITY if the lines are (more or less) collinear.
 */
static lineintersect_t
lineIntersect(
		const Vec2D<double> &pt, const Vec2D<double> &dt,
		const Vec2D<double> &pu, const Vec2D<double> &du) {
	lineintersect_t result;
	double denom=(dt.x*du.y-du.x*dt.y);
	if(abs(denom)<DBL_EPSILON) {
		result.t=INFINITY;
		result.u=INFINITY;
		return result;
	} else {
		Vec2D<double> dp=pu-pt;
		result.t=(dp.x*du.y-du.x*dp.y)/denom;
		result.u=(dt.x*dp.y-dp.x*dt.y)/denom;
		return result;
	}
}


/*
 * Calculate the distance from a given  point up to the wall,
 * measured in a given direction.
 * \param p A point somewhere in the world plane.
 * \param dir The direction vector.
 * \return The distance in units of the dir vector length
 *   or INFINITY if the ray does not hit a wall surface from the outside.
 */
double Wall::getDistance(const Vec2D<double> &p, const Vec2D<double> &dir) const {
	double result=INFINITY;
	{
		lineintersect_t l=lineIntersect(
				m_center+m_normal.rot90L()*m_length_2,
				m_normal, p, dir);
		if(fabs(l.t)<=m_width_2 && l.u>0.0 && l.u<result) result=l.u;
	}
	{
		lineintersect_t l=lineIntersect(
				m_center-m_normal.rot90L()*m_length_2,
				m_normal, p, dir);
		if(fabs(l.t)<=m_width_2 && l.u>0.0 && l.u<result) result=l.u;
	}
	{
		lineintersect_t l=lineIntersect(
				m_center+m_normal*m_width_2,
				m_normal.rot90L(), p, dir);
		if(fabs(l.t)<=m_length_2 && l.u>0.0 && l.u<result) result=l.u;
	}
	{
		lineintersect_t l=lineIntersect(
				m_center-m_normal*m_width_2,
				m_normal.rot90L(), p, dir);
		if(fabs(l.t)<=m_length_2 && l.u>0.0 && l.u<result) result=l.u;
	}
	return result;
}


geometry_msgs::Point Wall::getPoint(double p) const {
	return (m_center+m_normal.rot90L()*p*m_length_2).toPoint();
}

Vec2D<double> Wall::getPoint2(double p) const {
	return (m_center+m_normal.rot90L()*p*m_length_2);
}

double Wall::getThickness() const {
	return 2.0*m_width_2;
}

Wall::~Wall() {
	// TODO Auto-generated destructor stub
}

