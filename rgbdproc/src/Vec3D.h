/*
 * Vec3D.h
 *
 *  Created on: Oct 18, 2013
 *      Author: robo
 */

#ifndef VEC3D_H_
#define VEC3D_H_

#include <cmath>

template <typename T> class Vec3D {
public:
	T x, y, z;

	Vec3D(): x(0), y(0), z(0) {}

	Vec3D(T x, T y, T z): x(x), y(y), z(z)	{}

	~Vec3D() {}

	T operator *(const Vec3D<T> &b) const {
		return x*b.x+y*b.y+z*b.z;
	}
	Vec3D<T> operator*(const T &b) const {
		return Vec3D(x*b,y*b,z*b);
	}
	Vec3D<T> &operator *=(const T &b) {
		x*=b;
		y*=b;
		z*=b;
		return *this;
	}
	Vec3D<T> operator +(const Vec3D<T> &b) const {
		return Vec3D(x+b.x,y+b.y,z+b.z);
	}
	Vec3D<T> &operator +=(const Vec3D<T> &b) {
		x+=b.x;
		y+=b.y;
		z+=b.z;
		return *this;
	}
	Vec3D<T> operator -(const Vec3D<T> &b) const {
		return Vec3D(x-b.x,y-b.y,z-b.z);
	}
	Vec3D<T> &operator -=(const Vec3D<T> &b) {
		x-=b.x;
		y-=b.y;
		z-=b.z;
		return *this;
	}
/*
	Vec3D<T> rot90L() const{
		return Vec3D(-y,x);
	}
	Vec3D<T> rot90R() const{
		return Vec3D(y,-x);
	}
*/
	Vec3D<T> norm() const{
		double a=this->abs();
		return Vec3D(x/a,y/a,z/a);
	}
	T abs() const{
		return sqrt(x*(double)x+y*(double)y+z*(double)z);
	}
};

#endif /* VEC3D_H_ */
