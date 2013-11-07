/*
 * PointcloudProcessor.cpp
 *
 *  Created on: Nov 5, 2013
 *      Author: robo
 */

#include <OpenNI.h>
#include <cmath>
#include "PointcloudProcessor.h"
#include "MainWindow.h"
#include "PlaneEstimator.h"
#include <iostream>

PointcloudProcessor::PointcloudProcessor() :
	m_points(0),
	m_pointsAlloc(0),
	m_pointsLen(0),
	m_hFOV(57.7*M_PI/180.0),
	m_vFOV(45.0*M_PI/180.0),
	m_xlookup(0),
	m_ylookup(0),

	w(0), h(0),
	m_pixbuf(0),
	m_display(0)
{
	setCamOrientation(-21.99*M_PI/180.0,-2.2*M_PI/180.0,2720.0);
}

PointcloudProcessor::~PointcloudProcessor() {
	// TODO Auto-generated destructor stub
}

void PointcloudProcessor::onNewFrame(openni::VideoFrameRef& v) {
	if(!m_points) return;
	m_pointsLen=0;
	const void * const p_img=v.getData();
	const size_t linesz=v.getStrideInBytes();
	const double d_min=-m_camHeight*1.05;
	const double d_max=-m_camHeight*0.95;

	PlaneEstimator floor;

	uint8_t * const dst_img=m_pixbuf?gdk_pixbuf_get_pixels(m_pixbuf):0;
	const int dst_rowstride=m_pixbuf?gdk_pixbuf_get_rowstride(m_pixbuf):0;
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		uint8_t *dst_val=dst_img+y*dst_rowstride;
		for(int x=0; x<w; ++x) {
			uint16_t z=line[w-x-1];
			if(z==0) {
				if(m_pixbuf) {
					*dst_val++=0;
					*dst_val++=0;
					*dst_val++=0;
				}
				continue;
			}
			m_points[m_pointsLen].x=m_xlookup[x]*z;
			m_points[m_pointsLen].y=m_ylookup[y]*z;
			m_points[m_pointsLen].z=z;
			double d= m_points[m_pointsLen].x*m_floornormal[0]
					 +m_points[m_pointsLen].y*m_floornormal[1]
					 +m_points[m_pointsLen].z*m_floornormal[2];
			if(d<d_max && d>d_min) {
				floor.addPoint(m_points[m_pointsLen].x,m_points[m_pointsLen].y,z);
			}
			if(m_pixbuf) {
				uint8_t g;
				if(z>DIST_MAX) g=0;
				else if(z<DIST_MIN) g=255;
				else g=255-((unsigned int)z-DIST_MIN)
						*255U/(DIST_MAX-DIST_MIN);
				*dst_val++=g;
				if(d<-d_max && d>d_min) g=0;
				*dst_val++=g;
				*dst_val++=g;
			}
		}
	}
	if(floor.getNumPoints()>20000) {
		//std::cerr<<floor.getNumPoints()<<" Points; h="<<floor.getDistance()<<'\n';
		m_camHeight=floor.getDistance();
		m_floornormal=floor.getNormal();
	}
	if(m_display) {
		gdk_threads_enter();
		gtk_widget_queue_draw(m_display);
		gdk_threads_leave();
	}
}

void PointcloudProcessor::readStreamInfo(openni::VideoStream& v) {
	m_hFOV=v.getHorizontalFieldOfView();
	m_vFOV=v.getVerticalFieldOfView();
	w=v.getVideoMode().getResolutionX();
	h=v.getVideoMode().getResolutionY();
	if(m_display) gtk_widget_set_size_request(m_display,w,h);
	if(m_pixbuf) g_free(m_pixbuf);
	m_pixbuf=gdk_pixbuf_new(GDK_COLORSPACE_RGB,false,8,w,h);
	if(m_pointsAlloc < (size_t)(w*h)) {
		m_points=static_cast<point_t*>(realloc(m_points,w*h*sizeof(point_t)));
		m_pointsAlloc=w*h;
	}
	m_xlookup=static_cast<double*>(realloc(m_xlookup,w*sizeof(double)));
	m_ylookup=static_cast<double*>(realloc(m_ylookup,h*sizeof(double)));
	for(int x=0; x<w; ++x) m_xlookup[x]=(2.0*x/w-1.0)*tan(0.5*m_hFOV);
	for(int y=0; y<h; ++y) m_ylookup[y]=(2.0*y/h-1.0)*tan(0.5*m_vFOV);
}

void PointcloudProcessor::setCamOrientation(double upTilt, double rightTilt,
		double height) {
	m_camHeight=height;
	double us, uc, rs, rc;
	sincos(upTilt,&us,&uc);
	sincos(rightTilt,&rs,&rc);
	/*
	 * Rotation matrices (see Wikipedia: Euler angle):
	 * tilt to the right: [rc, rs, 0; -rs, rc, 0; 0, 0, 1]
	 * tilt upwards: [1, 0, 0; 0, uc, us; 0, -us, uc]
	 * In our mechanics, cam is first tilted left (bad camera hinge), then down.
	 * => floor normal [0; -1; 0] becomes [-rs; -rc*uc; rc*us]
	 */
	m_floornormal<<-rs, -rc*uc, rc*us;
}

PointcloudProcessor::camOrientation_t PointcloudProcessor::getCamOrientation() {
	camOrientation_t result;
	result.height=m_camHeight;
	result.rightTilt=asin(-m_floornormal[0]);
	result.upTilt=asin(m_floornormal[2]/sqrt(1-m_floornormal[0]*m_floornormal[0]));
	return result;
}

void PointcloudProcessor::setDisplay(GtkWidget* display) {
	if(m_display) {
		gtk_widget_set_size_request(m_display,m_prev_sizerq_w,m_prev_sizerq_h);
		gtk_widget_set_app_paintable(m_display,m_prev_app_paintable);
		gtk_widget_set_double_buffered(m_display,m_prev_double_buffered);
		g_signal_handler_disconnect(G_OBJECT(m_display),m_sighandler);
	}
	m_display=display;

	m_prev_app_paintable=gtk_widget_get_app_paintable(m_display);
	m_prev_double_buffered=gtk_widget_get_double_buffered(m_display);
	gtk_widget_get_size_request(m_display,&m_prev_sizerq_w,&m_prev_sizerq_h);

	gtk_widget_set_size_request(m_display,w,h);
	gtk_widget_set_app_paintable(m_display,TRUE);
	gtk_widget_set_double_buffered(m_display,FALSE);
	m_sighandler=g_signal_connect(G_OBJECT(m_display),"expose_event",
			G_CALLBACK(expose_event_callback),m_pixbuf);
}
