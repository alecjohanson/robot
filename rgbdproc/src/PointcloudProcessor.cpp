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
#include <stdint.h>

PointcloudProcessor::PointcloudProcessor() :
	m_points(0),
	m_pointsAlloc(0),
	m_hFOV(57.7*M_PI/180.0),
	m_vFOV(45.0*M_PI/180.0),
	m_xlookup(0),
	m_ylookup(0),

	w(0), h(0),
	m_pixbuf(0),
	m_display(0)
{
	setCamOrientation(-21.99*M_PI/180.0,-2.2*M_PI/180.0,2650.0);
}

PointcloudProcessor::~PointcloudProcessor() {
	// TODO Auto-generated destructor stub
}

double PointcloudProcessor::pxToPoint(point_t *pt,int x, int y, uint16_t z) {
	if(z==0||z>32767) {
		pt->p.setZero();
		pt->category=-1;
		return NAN;
	}
	pt->p[0]=m_xlookup[x]*z;
	pt->p[1]=m_ylookup[y]*z;
	pt->p[2]=z;
	pt->category=0;
	return (pt->p.cast<double>().dot(m_floornormal))+m_camHeight;
}

void PointcloudProcessor::onNewFrame(openni::VideoFrameRef& v) {
	if(!m_points) return;

	const void * const p_img=v.getData();
	const size_t linesz=v.getStrideInBytes();
	uint8_t * const dst_img=m_pixbuf?gdk_pixbuf_get_pixels(m_pixbuf):0;
	const int dst_rowstride=m_pixbuf?gdk_pixbuf_get_rowstride(m_pixbuf):0;

	PlaneEstimator floor;

	//convert depth image into point cloud in camera coordinates.
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		point_t *lineP=m_points+y*w;
		for(int x=0; x<w; ++x) {

			uint16_t z=line[w-x-1];
			double d=pxToPoint(lineP+x,x,y,z);
			if(z) {
				if(abs(d)<80.0) {
					floor.addPoint(lineP[x].p);
					lineP[x].category=1;
				}
				if(d>100.0 && d<1500.0)
					lineP[x].category=2;
			}
		}
	}

	//estimate floor plane
	if(floor.getNumPoints()>20000) {
		m_camHeight=floor.getDistance();
		m_floornormal=floor.getNormal();
		Eigen::Vector3d z(Eigen::Vector3d(0.,0.,1.)-m_floornormal*m_floornormal[2]);
		z.normalize();
		cam2robot.row(0)=z;
		cam2robot.row(1)=m_floornormal.cross(z);
		cam2robot.row(2)=m_floornormal;
		z=cam2robot*Eigen::Vector3d(m_xlookup[0],m_ylookup[h-1],1.);
		m_maxAngle=z[1]/z[0];
	}

	//convert points into floor coordinates
#define XHIST_RESOLUTION 256
	int16_t xhist[XHIST_RESOLUTION];
	memset(&xhist,-1,sizeof(xhist));
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		point_t *lineP=m_points+y*w;
		for(int x=0; x<w; ++x) {
			point_t *p=lineP+w-x-1;
			p->p=(cam2robot*p->p.cast<double>()).cast<int16_t>();
			if(p->category<=0) continue;
			int i=(((double)p->p[1]/(double)p->p[0])*(0.5/m_maxAngle)+0.5)*XHIST_RESOLUTION;
			if(i<0) i=0;
			else if(i>=XHIST_RESOLUTION) i=XHIST_RESOLUTION-1;
			if( (p->p[0]>xhist[i])
					|| (i>0 && p->p[0]>xhist[i-1])
					|| (i<XHIST_RESOLUTION-1 && p->p[0]>xhist[i+1]))
				xhist[i]=p->p[0];
		}
	}

	//remove object candidate points in the background (walls)
	unsigned int numCandidates=0;
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		point_t *lineP=m_points+y*w;
		for(int x=0; x<w; ++x) {
			point_t *p=lineP+w-x-1;
			if(p->category!=2) continue;
			int i=(((double)p->p[1]/(double)p->p[0])*(0.5/m_maxAngle)+0.5)*XHIST_RESOLUTION;
			if(i<0) i=0;
			else if(i>=XHIST_RESOLUTION) i=XHIST_RESOLUTION-1;
			if(xhist[i]-p->p[0]<500.) p->category=0;
			else ++numCandidates;
		}
	}

#define MAX_BBOX 32
	if(numCandidates>200) {
		bbox_t bboxes[MAX_BBOX];
		int numBbox=0;

		for(int y=1; y<h-1; ++y) {
			const uint16_t * const line=reinterpret_cast<const uint16_t *>(
					static_cast<const uint8_t *>(p_img)+y*linesz);
			point_t *lineP=m_points+y*w;
			for(int x=1; x<w-1; ++x) {
				if(lineP[x].category!=2) continue;
				if(lineP[x-1].category!=2 || lineP[x+1].category!=2) continue;
				if(lineP[x-w].category!=2 || lineP[x+w].category!=2) continue;
				int b=numBbox;
				for(int i=numBbox-1; i>=0; --i) {
					if(x-2>bboxes[i].img_xmax || x+2<bboxes[i].img_xmin) continue;
					if(y-2>bboxes[i].img_ymax || y+2<bboxes[i].img_ymin) continue;
					b=i;
					break;
				}
				if(b>=MAX_BBOX) break;
				if(b==numBbox) {
					++numBbox;
					bboxes[b].img_xmax=x+1;
					bboxes[b].img_xmin=x-1;
					bboxes[b].img_ymax=y+1;
					bboxes[b].img_ymin=y-1;
					bboxes[b].xmin=bboxes[b].xmax=lineP[x].p[0];
					bboxes[b].ymin=bboxes[b].ymax=lineP[x].p[1];
					bboxes[b].zmin=bboxes[b].zmax=lineP[x].p[2];
				} else {
					if(x>bboxes[b].img_xmax) bboxes[b].img_xmax=x;
					if(x<bboxes[b].img_xmin) bboxes[b].img_xmin=x;
					if(y>bboxes[b].img_ymax) bboxes[b].img_ymax=y;
					if(y<bboxes[b].img_ymin) bboxes[b].img_ymin=y;
					if(lineP[x].p[0]<bboxes[b].xmin) bboxes[b].xmin=lineP[x].p[0];
					if(lineP[x].p[0]>bboxes[b].xmax) bboxes[b].xmax=lineP[x].p[0];
					if(lineP[x].p[1]<bboxes[b].ymin) bboxes[b].ymin=lineP[x].p[1];
					if(lineP[x].p[1]>bboxes[b].ymax) bboxes[b].ymax=lineP[x].p[1];
					if(lineP[x].p[2]<bboxes[b].zmin) bboxes[b].zmin=lineP[x].p[2];
					if(lineP[x].p[2]>bboxes[b].zmax) bboxes[b].zmax=lineP[x].p[2];
				}
			}
		}
		int b=-1, bsize=0;
		for(int i=0; i<numBbox; ++i) {
			if(bboxes[i].zmin>180) continue;
			if(bboxes[i].zmax>1300) continue;
			if((bboxes[i].ymax-bboxes[i].ymin)<300) continue;
			if((bboxes[i].zmax-bboxes[i].zmin)<200) continue;
			int sz=(int)(bboxes[i].ymax-bboxes[i].ymin)
					* (int)(bboxes[i].zmax-bboxes[i].zmin);
			if(sz>bsize) {
				b=i;
				bsize=sz;
			}
		}
		if(b>=0) std::cerr<<bsize<<' '
				<<(bboxes[b].xmax-bboxes[b].xmin)<<'x'
				<<(bboxes[b].ymax-bboxes[b].ymin)<<'x'
				<<(bboxes[b].zmax-bboxes[b].zmin)<<'\n';
	}
	if(m_pixbuf) {
		for(int y=0; y<h; ++y) {
			const uint16_t * const line=reinterpret_cast<const uint16_t *>(
					static_cast<const uint8_t *>(p_img)+y*linesz);
			point_t *lineP=m_points+y*w;
			uint8_t *dst_val=dst_img+y*dst_rowstride;
			for(int x=0; x<w; ++x) {
				uint16_t z=line[w-x-1];
				uint8_t g;
				if(z>DIST_MAX) g=0;
				else if(z<DIST_MIN) g=255;
				else g=255-((unsigned int)z-DIST_MIN)
						*255U/(DIST_MAX-DIST_MIN);
				switch(lineP[x].category) {
				case -1:
					dst_val[0]=0; dst_val[1]=0; dst_val[2]=0;
					break;
				case 1:
					dst_val[0]=0; dst_val[1]=0; dst_val[2]=g;
					break;
				case 2:
					dst_val[0]=g; dst_val[1]=0; dst_val[2]=0;
					break;
				default:
					dst_val[0]=g; dst_val[1]=g; dst_val[2]=g;
					break;
				}
				dst_val+=3;
			}
		}
	}
	/*
	 * Now find a point cluster:
	 * -points above floor level
	 * -maximum height 150mm over floor
	 * -surrounded (left, right, bottom) by floor
	 * -more than 75% valid points
	 * -maximum BBox width 200mm
	 * -minimum BBox width, height: 30mm?
	 */

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
	for(int x=0; x<w; ++x) m_xlookup[x]=(2.0*x/(w-1)-1.0)*tan(0.5*m_hFOV);
	for(int y=0; y<h; ++y) m_ylookup[y]=(2.0*y/(h-1)-1.0)*tan(0.5*m_vFOV);
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
	/* m_floornormal is already a vector of doubles, but eclipse doesn't get it.
	 * The typecasts are just to keep eclipse happy and don't hurt. */
	result.rightTilt=asin((double)-m_floornormal[0]);
	result.upTilt=asin(
			(double)m_floornormal[2] /
			sqrt( 1.0 - (double)m_floornormal[0]*(double)m_floornormal[0] )
			);
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
