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
#ifndef NO_ROS
	#include <ros/ros.h>
#endif

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
	static const int16_t OBJ_MINH=300, OBJ_MINW=250;
	static const int16_t OBJ_MAXH=1400, OBJ_MAXW=1600;
	static const int16_t OBJ_MIN_WALLDIST=500;
	static const int16_t CAT_INVALID=-1, CAT_UNKNOWN=0, CAT_FLOOR=1,
			CAT_OBJCANDIDATE=2, CAT_OBJECT=3, CAT_WALL=4;
	static const size_t MIN_FLOOR_POINTS=20000;

	#ifndef NO_ROS
	ros::Publisher pub_img, pub_pointcloud;
	#endif

	class BoundingBox{
	public:
		int16_t img_xmin, img_xmax, img_ymin, img_ymax;
		int16_t xmin, xmax;
		int16_t ymin, ymax;
		int16_t zmin, zmax;
		BoundingBox();
		BoundingBox(int16_t x, int16_t y, const point_t *p);
		void setToPoint(int16_t x, int16_t y, const point_t *p);
		void addPoint(int16_t x, int16_t y, const point_t *p);
		bool isInside(const point_t *p);
		bool intersects(const BoundingBox &b);
	};
	class ColorBbox{
	public:
		int x1,x2,y1,y2;
		void set(double camHeight,
				const openni::VideoStream &d, const openni::VideoStream &c,
				const BoundingBox &bbox, const Eigen::Matrix3d cam2robot);
		bool isInside(int x, int y);
	};

	double pxToPoint(point_t *pt,int16_t x, int16_t y, uint16_t z);
};

#endif /* POINTCLOUDPROCESSOR_H_ */
