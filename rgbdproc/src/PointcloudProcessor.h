/*
 * PointcloudProcessor.h
 *
 *  Created on: Nov 5, 2013
 *      Author: robo
 */

#ifndef POINTCLOUDPROCESSOR_H_
#define POINTCLOUDPROCESSOR_H_

#include "Vec3D.h"
#include <gtk/gtk.h>

class PointcloudProcessor {
public:
	PointcloudProcessor();
	~PointcloudProcessor();
	void onNewFrame(openni::VideoFrameRef&);
	void readStreamInfo(openni::VideoStream&);
	void setDisplay(GtkWidget *);
	typedef struct {
		double upTilt;
		double rightTilt;
		double height;
	} camOrientation_t;
	void setCamOrientation(double upTilt, double rightTilt, double height);
	camOrientation_t getCamOrientation();
private:
	typedef struct {
		int16_t x,y,z;
		int16_t category;
	} point_t;
	point_t *m_points;
	size_t m_pointsAlloc, m_pointsLen;
	double m_hFOV, m_vFOV;
	double *m_xlookup, *m_ylookup;
	Vec3D<double> m_floornormal;
	double m_camHeight;

	int w,h;
	GdkPixbuf *m_pixbuf;
	GtkWidget *m_display;
	gboolean m_prev_app_paintable, m_prev_double_buffered;
	gint m_prev_sizerq_w, m_prev_sizerq_h;
	gulong m_sighandler;
	static const unsigned int DIST_MIN=3000, DIST_MAX=14000;
};

#endif /* POINTCLOUDPROCESSOR_H_ */
