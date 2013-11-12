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
	m_pointsAlloc(0),
	m_hFOV(57.7*M_PI/180.0),
	m_vFOV(45.0*M_PI/180.0),
	objectFrames(0),

	m_pixbuf(0),
	m_display(0)
{
	setCamOrientation(-21.99*M_PI/180.0,-2.2*M_PI/180.0,2650.0);
	//memset(m_points,0,sizeof(m_points));
	for(int x=0; x<320; ++x) m_xlookupSD[x]=(2.0*x/(320-1)-1.0)*tan(0.5*m_hFOV);
	for(int x=0; x<640; ++x) m_xlookupHD[x]=(2.0*x/(640-1)-1.0)*tan(0.5*m_hFOV);
	for(int y=0; y<240; ++y) m_ylookupSD[y]=(2.0*y/(240-1)-1.0)*tan(0.5*m_vFOV);
	for(int y=0; y<480; ++y) m_ylookupHD[y]=(2.0*y/(480-1)-1.0)*tan(0.5*m_vFOV);
}

PointcloudProcessor::~PointcloudProcessor() {
	// TODO Auto-generated destructor stub
}

bool PointcloudProcessor::objectDetected() const {
	return objectFrames>=MIN_OBJ;
}

inline double PointcloudProcessor::pxToPoint(point_t *pt,int16_t x, int16_t y, uint16_t z) {
	if(z==0||z>32767) {
		pt->p.setZero();
		pt->category=-1;
		return NAN;
	}
	pt->p[0]=x;
	pt->p[1]=y;
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
	int w=v.getVideoMode().getResolutionX();
	int h=v.getVideoMode().getResolutionY();

	PlaneEstimator floor;

	//convert depth image into point cloud in camera coordinates.
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		for(int x=0; x<w; ++x) {

			uint16_t z=line[w-x-1];
			double d;
			if(w>320)
				d=pxToPoint(&m_points[y][x],z*m_xlookupHD[x],z*m_ylookupHD[y],z);
			else
				d=pxToPoint(&m_points[y][x],z*m_xlookupSD[x],z*m_ylookupSD[y],z);
			if(z) {
				if(abs(d)<80.0) {
					floor.addPoint(m_points[y][x].p);
					m_points[y][x].category=1;
				}
				if(d>100.0 && d<1800.0)
					m_points[y][x].category=2;
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
		if(w>320)
			z=cam2robot*Eigen::Vector3d(m_xlookupHD[0],m_ylookupHD[h-1],1.);
		else
			z=cam2robot*Eigen::Vector3d(m_xlookupSD[0],m_ylookupSD[h-1],1.);
		m_maxAngle=z[1]/z[0];
	}

	//convert points into floor coordinates
#define XHIST_RESOLUTION 256
	int16_t xhist[XHIST_RESOLUTION];
	memset(&xhist,-1,sizeof(xhist));
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		for(int x=0; x<w; ++x) {
			point_t *p=&m_points[y][x];
			if(p->category<=0) continue;
			p->p=(cam2robot*p->p.cast<double>()).cast<int16_t>();
			p->p[2]+=m_camHeight;
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
		for(int x=0; x<w; ++x) {
			point_t *p=&m_points[y][x];
			if(p->category!=2) continue;
			int i=(((double)p->p[1]/(double)p->p[0])*(0.5/m_maxAngle)+0.5)*XHIST_RESOLUTION;
			if(i<0) i=0;
			else if(i>=XHIST_RESOLUTION) i=XHIST_RESOLUTION-1;
			if(xhist[i]-p->p[0]<500.) p->category=0;
			else ++numCandidates;
		}
	}
	//std::cerr<<numCandidates<<'\n';

#define MAX_BBOX 32
	if(numCandidates>200) {
		bbox_t bboxes[MAX_BBOX];
		int numBbox=0;

		for(int y=h-2; y>0; --y) {
			const uint16_t * const line=reinterpret_cast<const uint16_t *>(
					static_cast<const uint8_t *>(p_img)+y*linesz);
			for(int x=1; x<w-1; ++x) {
				if(m_points[y][x].category!=2) continue;
				int b=numBbox;
				for(int i=numBbox-1; i>=0; --i) {
					if(x-3>bboxes[i].img_xmax || x+3<bboxes[i].img_xmin) continue;
					if(y-3>bboxes[i].img_ymax || y+3<bboxes[i].img_ymin) continue;
					b=i;
					break;
				}
				if(b>=MAX_BBOX) break;
				if(b==numBbox) {
					if(m_points[y][x-1].category!=2 || m_points[y][x+1].category!=2) continue;
					if(m_points[y-1][x].category!=2 || m_points[y+1][x].category!=2) continue;
					++numBbox;
					bboxes[b].img_xmax=x+1;
					bboxes[b].img_xmin=x-1;
					bboxes[b].img_ymax=y+1;
					bboxes[b].img_ymin=y-1;
					bboxes[b].xmin=bboxes[b].xmax=m_points[y][x].p[0];
					bboxes[b].ymin=bboxes[b].ymax=m_points[y][x].p[1];
					bboxes[b].zmin=bboxes[b].zmax=m_points[y][x].p[2];
					updateBbox(&bboxes[b],&m_points[y][x-1]);
					updateBbox(&bboxes[b],&m_points[y][x+1]);
					updateBbox(&bboxes[b],&m_points[y-1][x]);
					updateBbox(&bboxes[b],&m_points[y+1][x]);
				} else {
					if(x>bboxes[b].img_xmax) bboxes[b].img_xmax=x;
					if(x<bboxes[b].img_xmin) bboxes[b].img_xmin=x;
					if(y>bboxes[b].img_ymax) bboxes[b].img_ymax=y;
					if(y<bboxes[b].img_ymin) bboxes[b].img_ymin=y;
					updateBbox(&bboxes[b],&m_points[y][x]);
				}
			}
		}
		int b=-1;
		int64_t bsize=0;
		for(int i=0; i<numBbox; ++i) {
			if(bboxes[i].zmin>160) continue;
			if(bboxes[i].zmax>1300) continue;
			if((bboxes[i].ymax-bboxes[i].ymin)<300) continue;
			if((bboxes[i].zmax-bboxes[i].zmin)<200) continue;
			if((bboxes[i].xmax-bboxes[i].xmin)<100) continue;
			if((bboxes[i].ymax-bboxes[i].ymin)>1000) continue;
			if((bboxes[i].xmax-bboxes[i].xmin)>1000) continue;
			int64_t sz=(int64_t)(bboxes[i].ymax-bboxes[i].ymin)
					* (int64_t)(bboxes[i].zmax-bboxes[i].zmin)
					* (int64_t)(bboxes[i].xmax-bboxes[i].xmin);
			if(sz>bsize) {
				b=i;
				bsize=sz;
			}
		}
		if(b>=0) {

			for(int y=bboxes[b].img_ymax+1;y<h;++y)
				for(int x=bboxes[b].img_xmin-1;x<=bboxes[b].img_xmax+1;++x) {
					if(m_points[y][x].category<0) continue;
					if(m_points[y][x].p[2]<0) continue;
					if(m_points[y][x].p[0]<bboxes[b].xmin-50
							|| m_points[y][x].p[0]>bboxes[b].xmax+50) continue;
					if(m_points[y][x].p[1]<bboxes[b].ymin-100
							|| m_points[y][x].p[1]>bboxes[b].ymax+100) continue;
					m_points[y][x].category=3;
					//updateBbox(&bboxes[b],&m_points[y][x]);
				}
			for(int y=bboxes[b].img_ymin; y<=bboxes[b].img_ymax; ++y) {
				for(int x=bboxes[b].img_xmin; x<=bboxes[b].img_xmax; ++x) {
					if(m_points[y][x].category==2) m_points[y][x].category=3;
				}
			}
			if(objectFrames<MIN_OBJ) ++objectFrames;
			else
			std::cerr<<bsize<<' '
					<<(bboxes[b].xmax-bboxes[b].xmin)<<'x'
					<<(bboxes[b].ymax-bboxes[b].ymin)<<'x'
					<<(bboxes[b].zmax-bboxes[b].zmin)<<'\n';
		} else {
			objectFrames=0;
		}
	}
	if(m_pixbuf) {
		for(int y=0; y<h; ++y) {
			const uint16_t * const line=reinterpret_cast<const uint16_t *>(
					static_cast<const uint8_t *>(p_img)+y*linesz);
			uint8_t *dst_val=dst_img+y*dst_rowstride;
			for(int x=0; x<w; ++x) {
				uint16_t z=line[w-x-1];
				uint8_t g;
				if(z>DIST_MAX) g=0;
				else if(z<DIST_MIN) g=255;
				else g=255-((unsigned int)z-DIST_MIN)
						*255U/(DIST_MAX-DIST_MIN);
				switch(m_points[y][x].category) {
				case -1:
					dst_val[0]=0; dst_val[1]=0; dst_val[2]=0;
					break;
				case 1:
					dst_val[0]=0; dst_val[1]=0; dst_val[2]=g;
					break;
				case 2:
					dst_val[0]=g; dst_val[1]=g; dst_val[2]=0;
					break;
				case 3:
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

	if(m_pixbuf) g_free(m_pixbuf);
	m_pixbuf=gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, 640,480);

	m_prev_app_paintable=gtk_widget_get_app_paintable(m_display);
	m_prev_double_buffered=gtk_widget_get_double_buffered(m_display);
	gtk_widget_get_size_request(m_display,&m_prev_sizerq_w,&m_prev_sizerq_h);

	gtk_widget_set_size_request(m_display,640,480);
	gtk_widget_set_app_paintable(m_display,TRUE);
	gtk_widget_set_double_buffered(m_display,FALSE);
	m_sighandler=g_signal_connect(G_OBJECT(m_display),"expose_event",
			G_CALLBACK(expose_event_callback),m_pixbuf);
}

inline void PointcloudProcessor::updateBbox(bbox_t* b, const point_t* p) {
	if(p->p[0]<b->xmin) b->xmin=p->p[0];
	if(p->p[0]>b->xmax) b->xmax=p->p[0];
	if(p->p[1]<b->ymin) b->ymin=p->p[1];
	if(p->p[1]>b->ymax) b->ymax=p->p[1];
	if(p->p[2]<b->zmin) b->zmin=p->p[2];
	if(p->p[2]>b->zmax) b->zmax=p->p[2];
}
