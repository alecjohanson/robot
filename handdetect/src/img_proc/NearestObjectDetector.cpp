/**
 * @file NearestObjectDetector.h
 * Description.
 * @author Christian Thießen
 * @copyright &copy; 2013 Christian Thießen
 * @date Oct 11, 2013
 */
/*
 *  Copyright 2013 Christian Thießen
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NearestObjectDetector.h"
#include <iostream>
#include <cmath>
#ifdef HAVE_GUI
#include "../MainWindow.h"
#endif
#ifdef HAVE_ROS
	#include <geometry_msgs/Point.h>
#endif

NearestObjectDetector::NearestObjectDetector(int w, int h):
	beta(M_PI/180.0*90.0),
	hcam(280.0),
	w(w),
	h(h),
#ifdef HAVE_GUI
	m_target(0),
	m_prev_app_paintable(FALSE), m_prev_double_buffered(FALSE),
	m_prev_sizerq_w(0), m_prev_sizerq_h(0),
	m_sighandler(0)
#endif
#ifdef HAVE_ROS
	m_rosnode()
#endif
{
	sincos(beta,&sinb,&cosb);
	m_mask=new uint8_t[h*w];
#ifdef HAVE_ROS
	m_pointpub=m_rosnode.advertise<geometry_msgs::Point>("milestone0/object",1);
#endif
#ifdef HAVE_GUI
	m_pixbuf=gdk_pixbuf_new(GDK_COLORSPACE_RGB,false,8,w,h);
#endif
}

NearestObjectDetector::~NearestObjectDetector() {
	delete m_mask;
}

#ifdef HAVE_GUI
void NearestObjectDetector::setTarget(GtkWidget* target) {
	if(m_target) {
		gtk_widget_set_size_request(m_target,m_prev_sizerq_w,m_prev_sizerq_h);
		gtk_widget_set_app_paintable(m_target,m_prev_app_paintable);
		gtk_widget_set_double_buffered(m_target,m_prev_double_buffered);
		g_signal_handler_disconnect(G_OBJECT(m_target),m_sighandler);
	}
	m_target=target;

	m_prev_app_paintable=gtk_widget_get_app_paintable(m_target);
	m_prev_double_buffered=gtk_widget_get_double_buffered(m_target);
	gtk_widget_get_size_request(m_target,&m_prev_sizerq_w,&m_prev_sizerq_h);

	gtk_widget_set_size_request(m_target,w,h);
	gtk_widget_set_app_paintable(m_target,TRUE);
	gtk_widget_set_double_buffered(m_target,FALSE);
	m_sighandler=g_signal_connect(G_OBJECT(m_target),"expose_event",
			G_CALLBACK(expose_event_callback),m_pixbuf);
}
#endif

void NearestObjectDetector::onNewFrame(openni::VideoStream& v) {
	static int nFrame=INT_MAX;
	if(nFrame++<2) return;
	else nFrame=0;

	static int numDetect=0;

	openni::VideoFrameRef f;
	v.readFrame(&f);
	int w=f.getWidth();
	int h=f.getHeight();

	const uint16_t *src_img=static_cast<const uint16_t*>(f.getData());
	uint16_t minDist=65535;

	//magic constants for coordinate transformation
	const double tanax=-tan(v.getHorizontalFieldOfView()*0.5);
	const double tanay=tan(v.getVerticalFieldOfView()*0.5);
	static const double f_xr_zs=fma(tanay,cosb,sinb);
	static const double f_xr_zs_y=-2.0/f.getHeight()*tanay*cosb;
	static const double f_yr_zs_x=-2.0/f.getWidth()*tanax;
	static const double f_zr_zs=fma(tanay,sinb,-cosb);
	static const double f_zr_zs_y=-2.0/f.getHeight()*tanay*sinb;

	/*
	 * Select all points that are not too close and high enough over the ground.
	 */
	unsigned int zerocount=0;
	for(int y=0; y<h;++y) {
		const uint16_t *src_line=src_img+(y*f.getStrideInBytes()
				/sizeof(uint16_t));
		double row_z_y=fma(y,f_zr_zs_y,f_zr_zs);
		for(int x=0; x<w;++x) {
			double zs=src_line[x]*0.1; //converts 100um units to 1mm
			double z=fma(zs,row_z_y,hcam);
			if(src_line[x]==0) ++zerocount;
			m_mask[y*w+x]=(z>Z_MIN && src_line[x]>=DIST_MIN)
					? 1:0;
			if(m_mask[y*w+x] && src_line[x]<minDist) minDist=src_line[x];
		}
	}

	//calculate center of mass for these points
	int64_t xsum=0, ysum=0, count=0;
	int64_t xcsum=0, ycsum=0;
	if(minDist<DIST_MAX && zerocount<(w*h/2)) {
		for(int y=0; y<h;++y) {
			const uint16_t *src_line=src_img+(y*f.getStrideInBytes()
					/sizeof(uint16_t));
			for(int x=0; x<w;++x) {
				if(m_mask[y*w+x]) {
					if(src_line[x]<minDist+DIST_THRESH) {
						xsum+=x;
						ysum+=y;
						++count;
					} else {
						m_mask[y*w+x]=0;
					}
				}
			}
		}

		//if we found some points, calculate mean square distance from center
		if(count>0) {
			xsum/=count;
			ysum/=count;
			for(int y=0; y<h;++y) {
				for(int x=0; x<w;++x) {
					if(m_mask[y*w+x]) {
						xcsum+=(x-xsum)*(x-xsum);
						ycsum+=(y-ysum)*(y-ysum);
					}
				}
			}
			xcsum=sqrt(xcsum/count);
			ycsum=sqrt(ycsum/count);
		}

	//if the detected blob is too small, too large or too distant
	//	set it to 0 width
	} else xcsum=0;
	if(xcsum>w/3 || xcsum<w/64 || ycsum<h/128) xcsum=0;

	//require 3 nonzero blobs in a row before processing them
	if(xcsum>0) numDetect++;
	else         numDetect=0;
	if(numDetect<3)	{
		xsum=-1; ysum=-1;
		xcsum=0; ycsum=0;
	}

#ifdef HAVE_ROS
	geometry_msgs::Point p;
	static int num_fail=0;
	if(xcsum>0) {
		double row_x=fma(ysum,f_xr_zs_y,f_xr_zs);
		double row_z_y=fma(ysum,f_zr_zs_y,f_zr_zs);
		double zs=minDist*0.1; //converts 100um units to 1mm
		p.x=zs*row_x;
		p.y=zs*fma(xsum,f_yr_zs_x,tanax);
		p.z=fma(zs,row_z_y,h);
		m_pointpub.publish(p);
		num_fail=0;
	} else {
		p.x=NAN;
		p.y=NAN;
		p.z=NAN;
		if(num_fail>10 || zerocount>(w*h/2))
			m_pointpub.publish(p);
		else
			num_fail++;
	}
#endif

//draw the depth image and a red square for the detected object
#ifdef HAVE_GUI
	if(!m_target) {
		f.release();
		return;
	}
	uint8_t *dst_img=gdk_pixbuf_get_pixels(m_pixbuf);

	int x1=xsum-xcsum, y1=ysum-ycsum, x2=xsum+xcsum, y2=ysum+ycsum;
	//convert depth image to RGB grey image in the pixbuf
	for(int y=0; y<h;++y) {
		const uint16_t *src_line=src_img+(y*f.getStrideInBytes()
				/sizeof(uint16_t));
		uint8_t *dst_val=dst_img+y*gdk_pixbuf_get_rowstride(m_pixbuf);
		for(int x=0; x<w;++x) {
			uint8_t g;
			//sif(src_line[x]==0) g=0; else
			if(src_line[x]<DIST_MIN) g=255;
			else if(src_line[x]>DIST_MAX) g=0;
			else g=255-((unsigned int)src_line[x]-DIST_MIN)
					*255U/(DIST_MAX-DIST_MIN);
			uint8_t r=g;
			if(x>=x1 && x<=x2 && y>=y1 && y<=y2)
				if(x<=x1+1||x>=x2-1||y<=y1+1||y>=y2-1) {
					g=0; r=192;
				}
			*dst_val++=r;
			*dst_val++=g;
			*dst_val++=g;
		}
	}
#endif
	f.release();
#ifdef HAVE_GUI
	gdk_threads_enter();
	gtk_widget_queue_draw(m_target);
	gdk_threads_leave();
#endif
}

