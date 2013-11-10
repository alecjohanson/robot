/*
 * PointcloudProcessor.h
 *
 *  Created on: Nov 5, 2013
 *      Author: robo
 */

#ifndef POINTCLOUDPROCESSOR_H_
#define POINTCLOUDPROCESSOR_H_

#include <Eigen/Core>
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
		Eigen::Matrix<int16_t,3,1> p;
		int16_t category;
	} point_t;
	typedef struct {
		uint16_t img_xmin, img_xmax, img_ymin, img_ymax;
		int16_t xmin, xmax;
		int16_t ymin, ymax;
		int16_t zmin, zmax;
	} bbox_t;
	point_t *m_points;
	size_t m_pointsAlloc;
	double m_hFOV, m_vFOV;
	double *m_xlookup, *m_ylookup;
	Eigen::Vector3d m_floornormal;
	Eigen::Matrix<double,3,3,Eigen::RowMajor> cam2robot;
	double m_camHeight;
	double m_maxAngle;

	int w,h;
	GdkPixbuf *m_pixbuf;
	GtkWidget *m_display;
	gboolean m_prev_app_paintable, m_prev_double_buffered;
	gint m_prev_sizerq_w, m_prev_sizerq_h;
	gulong m_sighandler;
	static const unsigned int DIST_MIN=3000, DIST_MAX=14000;

	double pxToPoint(point_t *pt,int x, int y, uint16_t z);
};

#endif /* POINTCLOUDPROCESSOR_H_ */
