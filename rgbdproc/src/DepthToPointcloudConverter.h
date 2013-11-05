/*
 * DepthToPointcloudConverter.h
 *
 *  Created on: Nov 5, 2013
 *      Author: robo
 */

#ifndef DEPTHTOPOINTCLOUDCONVERTER_H_
#define DEPTHTOPOINTCLOUDCONVERTER_H_


class DepthToPointcloudConverter {
public:
	DepthToPointcloudConverter();
	virtual ~DepthToPointcloudConverter();
	virtual void onNewFrame(openni::VideoFrameRef&);
	virtual void readStreamInfo(openni::VideoStream&);
private:
	typedef struct {
		int16_t x,y,z;
		int16_t category;
	} point_t;
	point_t *m_points;
	double *m_xlookup, *m_ylookup;
	size_t m_pointsAlloc, m_pointsLen;
	double m_hFOV, m_vFOV;
};

#endif /* DEPTHTOPOINTCLOUDCONVERTER_H_ */
