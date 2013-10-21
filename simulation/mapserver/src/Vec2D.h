/*
 * Vec2D.h
 *
 *  Created on: Oct 18, 2013
 *      Author: robo
 */

#ifndef VEC2D_H_
#define VEC2D_H_

#include <cmath>
#include <geometry_msgs/Point.h>

template <typename T> class Vec2D {
public:
	T x, y;

	Vec2D(): x(0), y(0) {}

	Vec2D(T x, T y): x(x), y(y)	{}

	~Vec2D() {}

	T operator *(const Vec2D<T> &b) const {
		return x*b.x+y*b.y;
	}
	Vec2D<T> operator*(const T &b) const {
		return Vec2D(x*b,y*b);
	}
	Vec2D<T> &operator *=(const T &b) {
		x*=b;
		y*=b;
		return *this;
	}
	Vec2D<T> operator +(const Vec2D<T> &b) const {
		return Vec2D(x+b.x,y+b.y);
	}
	Vec2D<T> &operator +=(const Vec2D<T> &b) {
		x+=b.x;
		y+=b.y;
		return *this;
	}
	Vec2D<T> operator -(const Vec2D<T> &b) const {
		return Vec2D(x-b.x,y-b.y);
	}
	Vec2D<T> &operator -=(const Vec2D<T> &b) {
		x-=b.x;
		y-=b.y;
		return *this;
	}
	Vec2D<T> rot90L() const{
		return Vec2D(-y,x);
	}
	Vec2D<T> rot90R() const{
		return Vec2D(y,-x);
	}
	Vec2D<T> norm() const{
		double a=this->abs();
		return Vec2D(x/a,y/a);
	}
	T abs() const{
		return hypot(x,y);
	}
	geometry_msgs::Point toPoint() const {
		geometry_msgs::Point r;
		r.x=x;
		r.y=y;
		r.z=0;
		return r;
	}
};

#endif /* VEC2D_H_ */
