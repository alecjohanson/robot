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
	m_vFOV(45.0*M_PI/180.0)
{
	// TODO Auto-generated constructor stub

}

DepthToPointcloudConverter::~DepthToPointcloudConverter() {
	// TODO Auto-generated destructor stub
}

void DepthToPointcloudConverter::onNewFrame(openni::VideoFrameRef& v) {
	int resX=v.getWidth();
	int resY=v.getHeight();
	if(m_pointsAlloc < resX*resY) {
		m_points=realloc(m_points,resX*resY);
		m_pointsAlloc=resX*resY;
	}
	m_pointsLen=0;
	void *p_img=v.getData();
	size_t linesz=v.getStrideInBytes();
	for(int y=0; y<resY; ++y) {
		uint16_t *line=static_cast<uint16_t *>(p_img+y*linesz);
		for(int x=0; x<resX; ++x) {
			if(line[x]==0) continue;
			m_points[m_pointsLen].x=(2.0*x/resX-1.0)*tan(0.5*m_hFOV)*line[x];
			//todo: m_xlookup[x]*line[x];
			m_points[m_pointsLen].y=(2.0*y/resY-1.0)*tan(0.5*m_hFOV)*line[x];
			//todo: m_ylookup[y]*line[x];
			m_points[m_pointsLen].z=line[x];
		}
	}
}

virtual void DepthToPointcloudConverter::readStreamInfo(openni::VideoStream& v) {
	m_hFOV=v.getHorizontalFieldOfView();
	m_vFOV=v.getVerticalFieldOfView();
	int resX=v.getVideoMode().getResolutionX();
	int resY=v.getVideoMode().getResolutionY();
	if(m_pointsAlloc < resX*resY) {
		m_points=realloc(m_points,resX*resY*sizeof(point_t));
		m_pointsAlloc=resX*resY;
	}
	m_xlookup=realloc(m_xlookup,resX*sizeof(double));
	m_ylookup=realloc(m_ylookup,resY*sizeof(double));
	for(int i=0; i<resX; ++i) m_xlookup[i]=

}
