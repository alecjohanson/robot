/*
 * DepthToPointcloudConverter.cpp
 *
 *  Created on: Nov 5, 2013
 *      Author: robo
 */

#include <OpenNI.h>
#include <cmath>
#include "DepthToPointcloudConverter.h"

DepthToPointcloudConverter::DepthToPointcloudConverter() :
	m_points(0),
	m_pointsAlloc(0),
	m_pointsLen(0),
	m_hFOV(57.7*M_PI/180.0),
	m_vFOV(45.0*M_PI/180.0),
	m_xlookup(0),
	m_ylookup(0)
{
	// TODO Auto-generated constructor stub

}

DepthToPointcloudConverter::~DepthToPointcloudConverter() {
	// TODO Auto-generated destructor stub
}

void DepthToPointcloudConverter::onNewFrame(openni::VideoFrameRef& v) {
	if(!m_points) return;
	m_pointsLen=0;
	const int resX=v.getWidth();
	const int resY=v.getHeight();
	const void * const p_img=v.getData();
	const size_t linesz=v.getStrideInBytes();
	for(int y=0; y<resY; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		for(int x=0; x<resX; ++x) {
			if(line[x]==0) continue;
			m_points[m_pointsLen].x=m_xlookup[x]*line[x];
			m_points[m_pointsLen].y=m_ylookup[y]*line[x];
			m_points[m_pointsLen].z=line[x];
		}
	}
}

void DepthToPointcloudConverter::readStreamInfo(openni::VideoStream& v) {
	m_hFOV=v.getHorizontalFieldOfView();
	m_vFOV=v.getVerticalFieldOfView();
	int resX=v.getVideoMode().getResolutionX();
	int resY=v.getVideoMode().getResolutionY();
	if(m_pointsAlloc < (size_t)(resX*resY)) {
		m_points=static_cast<point_t*>(realloc(m_points,resX*resY*sizeof(point_t)));
		m_pointsAlloc=resX*resY;
	}
	m_xlookup=static_cast<double*>(realloc(m_xlookup,resX*sizeof(double)));
	m_ylookup=static_cast<double*>(realloc(m_ylookup,resY*sizeof(double)));
	for(int x=0; x<resX; ++x) m_xlookup[x]=(2.0*x/resX-1.0)*tan(0.5*m_hFOV);
	for(int y=0; y<resY; ++y) m_ylookup[y]=(2.0*y/resY-1.0)*tan(0.5*m_vFOV);
}
