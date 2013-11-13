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
	void onNewFrame(openni::VideoFrameRef&, openni::VideoFrameRef&);
	void readStreamInfo(openni::VideoStream&, openni::VideoStream&);
	void setDisplay(GtkWidget *);
	typedef struct {
		double upTilt;
		double rightTilt;
		double height;
	} camOrientation_t;
	void setCamOrientation(double upTilt, double rightTilt, double height);
	camOrientation_t getCamOrientation();
	bool objectDetected() const;
private:
	openni::VideoStream *s_depth, *s_color;
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
	point_t m_points[480][640];
	size_t m_pointsAlloc;
	double m_hFOV, m_vFOV;
	double m_xlookupSD[320], m_ylookupSD[240];
	double m_xlookupHD[640], m_ylookupHD[480];
	Eigen::Vector3d m_floornormal;
	Eigen::Matrix<double,3,3,Eigen::RowMajor> cam2robot;
	double m_camHeight;
	double m_maxAngle;
	int objectFrames;

	int w,h;
	GdkPixbuf *m_pixbuf;
	GtkWidget *m_display;
	gboolean m_prev_app_paintable, m_prev_double_buffered;
	gint m_prev_sizerq_w, m_prev_sizerq_h;
	gulong m_sighandler;
	static const unsigned int DIST_MIN=3000, DIST_MAX=14000;
	static const int MIN_OBJ=3;

	double pxToPoint(point_t *pt,int16_t x, int16_t y, uint16_t z);
	void updateBbox(bbox_t *b, const point_t *p);
};

#endif /* POINTCLOUDPROCESSOR_H_ */
