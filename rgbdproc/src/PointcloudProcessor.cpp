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
#ifndef NO_ROS
	#include <sensor_msgs/Image.h>
	#include <sensor_msgs/image_encodings.h>
	#include <sensor_msgs/PointCloud2.h>
#endif

PointcloudProcessor::PointcloudProcessor() :
	s_depth(0),
	s_color(0),
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
#ifndef NO_ROS
	ros::NodeHandle nh;
	pub_img=nh.advertise<sensor_msgs::Image>("objdetect/images",1);
	pub_pointcloud=nh.advertise<sensor_msgs::PointCloud2>("objdetect/pointclouds",1);
#endif

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
		pt->category=CAT_INVALID;
		return NAN;
	}
	pt->p[0]=x;
	pt->p[1]=y;
	pt->p[2]=z;
	pt->category=CAT_UNKNOWN;
	return (pt->p.cast<double>().dot(m_floornormal))+m_camHeight;
}

void PointcloudProcessor::onNewFrame(openni::VideoFrameRef& d,openni::VideoFrameRef& c) {
	if(!m_points) return;

	const void * const p_img=d.getData();
	const size_t linesz=d.getStrideInBytes();
	uint8_t * const dst_img=m_pixbuf?gdk_pixbuf_get_pixels(m_pixbuf):0;
	const int dst_rowstride=m_pixbuf?gdk_pixbuf_get_rowstride(m_pixbuf):0;
	int w=d.getVideoMode().getResolutionX();
	int h=d.getVideoMode().getResolutionY();

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
			if(m_points[y][x].category!=CAT_INVALID) {
				if(abs(d)<80.0) {
					floor.addPoint(m_points[y][x].p);
					m_points[y][x].category=CAT_FLOOR;
				} else if(d>100.0 && d<OBJ_MAXH) {
					m_points[y][x].category=CAT_OBJCANDIDATE;
				} else if(d>OBJ_MAXH+100) {
					m_points[y][x].category=CAT_WALL;
				}
			}
		}
	}

	//estimate floor plane
	if(floor.getNumPoints()>=MIN_FLOOR_POINTS) {
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
#define XHIST_RESOLUTION 128
	float xhist[XHIST_RESOLUTION];
	memset(&xhist,0,sizeof(xhist));
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		for(int x=0; x<w; ++x) {
			point_t *p=&m_points[y][x];
			if(p->category<=CAT_INVALID) continue;

			Eigen::Vector3d pd=cam2robot*p->p.cast<double>();
			p->p=pd.cast<int16_t>();
			p->p[2]+=m_camHeight;

//			if(p->category!=CAT_WALL && p->category!=CAT_FLOOR) continue;
			if(p->category!=CAT_FLOOR) continue;

			int i=((pd[1]/pd[0])*(0.5/m_maxAngle)+0.5)*XHIST_RESOLUTION;
			if(i<0) i=0;
			else if(i>=XHIST_RESOLUTION) i=XHIST_RESOLUTION-1;

			float d=hypot((double)pd[0],(double)pd[1]);
			if(d>xhist[i]) xhist[i]=d;
		}
	}

	//remove object candidate points in the background (walls)
	unsigned int numCandidates=0;
	for(int y=0; y<h; ++y) {
		const uint16_t * const line=reinterpret_cast<const uint16_t *>(
				static_cast<const uint8_t *>(p_img)+y*linesz);
		for(int x=0; x<w; ++x) {
			point_t *p=&m_points[y][x];

			if(p->category!=CAT_OBJCANDIDATE) continue;

			int i=(((double)p->p[1]/(double)p->p[0])*(0.5/m_maxAngle)+0.5)*XHIST_RESOLUTION;
			if(i<0) i=0; else if(i>=XHIST_RESOLUTION) i=XHIST_RESOLUTION-1;

			float d=hypot((double)p->p[0],(double)p->p[1]);
			if((xhist[i]-d)<OBJ_MIN_WALLDIST)
				p->category=CAT_UNKNOWN;
			if(i>0 && (xhist[i-1]-d)<OBJ_MIN_WALLDIST)
				p->category=CAT_UNKNOWN;
			if(i<XHIST_RESOLUTION-1 && (xhist[i]-d)<OBJ_MIN_WALLDIST)
				p->category=CAT_UNKNOWN;

			if(p->category==CAT_OBJCANDIDATE) ++numCandidates;
		}
	}
	std::cerr<<numCandidates<<'\n';

#define MAX_BBOX 32
	BoundingBox finalBbox;
	if(numCandidates>200) {
		BoundingBox bboxes[MAX_BBOX];
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
					bboxes[b].setToPoint(x,y,&m_points[y][x]);
					bboxes[b].addPoint(x-1,y,&m_points[y][x-1]);
					bboxes[b].addPoint(x+1,y,&m_points[y][x+1]);
					bboxes[b].addPoint(x,y-1,&m_points[y-1][x]);
					bboxes[b].addPoint(x,y+1,&m_points[y+1][x]);
				} else {
					bboxes[b].addPoint(x,y,&m_points[y][x]);
				}
			}
		}
		int b=-1;
		int64_t bsize=0;
		for(int i=0; i<numBbox; ++i) {
			if(bboxes[i].zmin>160) continue;
			if(bboxes[i].zmax>OBJ_MAXH) continue;
			if((bboxes[i].ymax-bboxes[i].ymin)<OBJ_MINW) continue;
			if((bboxes[i].zmax-bboxes[i].zmin)<OBJ_MINH) continue;
			if((bboxes[i].xmax-bboxes[i].xmin)<OBJ_MINW/3) continue;
			if((bboxes[i].ymax-bboxes[i].ymin)>OBJ_MAXW) continue;
			if((bboxes[i].xmax-bboxes[i].xmin)>OBJ_MAXW) continue;
			int64_t sz=(int64_t)(bboxes[i].ymax-bboxes[i].ymin)
					* (int64_t)(bboxes[i].zmax-bboxes[i].zmin)
					* (int64_t)(bboxes[i].xmax-bboxes[i].xmin);
			if(sz>bsize) {
				b=i;
				bsize=sz;
			}
		}
		numCandidates=0;
		if(b>=0) {
			finalBbox=bboxes[b];
			for(int y=bboxes[b].img_ymax+1;y<h;++y)
				for(int x=bboxes[b].img_xmin-1;x<=bboxes[b].img_xmax+1;++x) {
					if(m_points[y][x].category<0) continue;
					if(m_points[y][x].p[2]<0) continue;
					if(m_points[y][x].p[0]<bboxes[b].xmin-50
							|| m_points[y][x].p[0]>bboxes[b].xmax+50) continue;
					if(m_points[y][x].p[1]<bboxes[b].ymin-100
							|| m_points[y][x].p[1]>bboxes[b].ymax+100) continue;
					m_points[y][x].category=3;
					++numCandidates;
					finalBbox.addPoint(x,y,&m_points[y][x]);
				}
			for(int y=finalBbox.img_ymin; y<=finalBbox.img_ymax; ++y) {
				for(int x=finalBbox.img_xmin; x<=finalBbox.img_xmax; ++x) {
					if(m_points[y][x].category==2) {
						m_points[y][x].category=3;
						++numCandidates;
					}
				}
			}
			if(objectFrames<MIN_OBJ) {
				++objectFrames;
			}
		} else { //if(b>0)
			objectFrames=0;
		}
	} else { //if(numCandidates>200)
		objectFrames=0;
	}
	if(objectFrames<MIN_OBJ) numCandidates=0;

	ColorBbox color_bbox;
	if(c.isValid() && numCandidates) {
		color_bbox.set(m_camHeight,*s_depth,*s_color,finalBbox,cam2robot);
	}
#ifndef NO_ROS
	if(numCandidates) {
		if(pub_pointcloud.getNumSubscribers()>0) {
			sensor_msgs::PointCloud2 msg_pcl;
			msg_pcl.header.stamp=ros::Time::now();
			msg_pcl.header.frame_id="base_link";
			sensor_msgs::PointField pf;
			pf.datatype=sensor_msgs::PointField::FLOAT32;
			pf.count=1;
			pf.name="x";
			pf.offset=0;
			msg_pcl.fields.push_back(pf);
			pf.name="y";
			pf.offset=4;
			msg_pcl.fields.push_back(pf);
			pf.name="z";
			pf.offset=8;
			msg_pcl.fields.push_back(pf);
			msg_pcl.point_step=16;
			msg_pcl.is_bigendian=0;
			msg_pcl.width=numCandidates;
			msg_pcl.height=1;
			msg_pcl.is_dense=true;
			msg_pcl.row_step=msg_pcl.point_step*msg_pcl.width;
			msg_pcl.data.resize(msg_pcl.row_step*msg_pcl.height);

			uint8_t *p=&msg_pcl.data.front();
			for(int y=finalBbox.img_ymin; y<=finalBbox.img_ymax; ++y) {
				for(int x=finalBbox.img_xmin; x<=finalBbox.img_xmax; ++x) {
					if(m_points[y][x].category==3) {
						float v;
						v=m_points[y][x].p[0]*0.0001f;
						memcpy(p,&v,4);
						p+=4;
						v=m_points[y][x].p[1]*0.0001f;
						memcpy(p,&v,4);
						p+=4;
						v=m_points[y][x].p[2]*0.0001f;
						memcpy(p,&v,4);
						p+=8;
					}
				}
			}
			std::cerr<<numCandidates<<' '<<(p-&msg_pcl.data.front())/6<<'\n';
			pub_pointcloud.publish(msg_pcl);
		}
		if(c.isValid() && pub_img.getNumSubscribers()>0){//send color images
			sensor_msgs::Image msg_img;
			msg_img.header.stamp=ros::Time::now();
			msg_img.height=1+color_bbox.y2-color_bbox.y1;
			msg_img.width=1+color_bbox.x2-color_bbox.x1;
			msg_img.encoding=sensor_msgs::image_encodings::RGB8;
			msg_img.is_bigendian=0;
			msg_img.step=msg_img.width*3;
			msg_img.data.reserve(msg_img.step*msg_img.height*3);
			int cw=s_color->getVideoMode().getResolutionX();
			for(int y=color_bbox.y1; y<=color_bbox.y2; ++y) {
				const uint8_t *cpx=static_cast<const uint8_t*>(c.getData())+y*c.getStrideInBytes()+3*(cw-color_bbox.x1-1);
				for(int x=color_bbox.x1; x<=color_bbox.x2; ++x) {
					msg_img.data.push_back(cpx[2]);
					msg_img.data.push_back(cpx[1]);
					msg_img.data.push_back(cpx[0]);
					cpx-=3;
				}
			}
			pub_img.publish(msg_img);
		}
	}
#endif
	if(m_pixbuf) {
		if(c.isValid() && finalBbox.img_ymax) {
			for(int y=0; y<h; ++y) {
				const uint16_t * const line=reinterpret_cast<const uint16_t *>(
						static_cast<const uint8_t *>(p_img)+y*linesz);
				uint8_t *dst_val=dst_img+y*dst_rowstride;
				const uint8_t *colorLine=static_cast<const uint8_t*>(c.getData())+y*c.getStrideInBytes()+3*w-3;
				for(int x=0; x<w; ++x) {
					if(y<color_bbox.y1 || y>color_bbox.y2
							|| x<color_bbox.x1 || x>color_bbox.x2) {
						dst_val[0]=colorLine[0]/2;
						dst_val[1]=colorLine[1]/2;
						dst_val[2]=colorLine[2]/2;
					} else {
						dst_val[0]=colorLine[0];
						dst_val[1]=colorLine[1];
						dst_val[2]=colorLine[2];
					}
					dst_val+=3;
					colorLine-=3;
				}
			}
		} else {
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
	}

	if(m_display) {
		gdk_threads_enter();
		gtk_widget_queue_draw(m_display);
		gdk_threads_leave();
	}
}

void PointcloudProcessor::readStreamInfo(openni::VideoStream& d, openni::VideoStream& c) {
	m_hFOV=d.getHorizontalFieldOfView();
	m_vFOV=d.getVerticalFieldOfView();
	s_depth=&d;
	s_color=&c;
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

	gtk_widget_set_size_request(m_display,320,240);
	gtk_widget_set_app_paintable(m_display,TRUE);
	gtk_widget_set_double_buffered(m_display,FALSE);
	m_sighandler=g_signal_connect(G_OBJECT(m_display),"expose_event",
			G_CALLBACK(expose_event_callback),m_pixbuf);
}

void PointcloudProcessor::BoundingBox::addPoint(int16_t x, int16_t y, const point_t* p) {
	if(x<img_xmin) img_xmin=x;
	if(x>img_xmax) img_xmax=x;
	if(y<img_ymin) img_ymin=y;
	if(y>img_ymax) img_ymax=y;
	if(p->p[0]<xmin) xmin=p->p[0];
	if(p->p[0]>xmax) xmax=p->p[0];
	if(p->p[1]<ymin) ymin=p->p[1];
	if(p->p[1]>ymax) ymax=p->p[1];
	if(p->p[2]<zmin) zmin=p->p[2];
	if(p->p[2]>zmax) zmax=p->p[2];
}

bool PointcloudProcessor::BoundingBox::isInside(const point_t* p) {
	if(p->p[0]<xmin || p->p[0]>xmax) return false;
	if(p->p[1]<ymin || p->p[1]>ymax) return false;
	if(p->p[2]<zmin || p->p[2]>zmax) return false;
	return true;
}

PointcloudProcessor::BoundingBox::BoundingBox(int16_t x, int16_t y,
		const point_t* p)
:
		img_xmin(x), img_xmax(x),
		img_ymin(y), img_ymax(y),
		xmin(p->p[0]), xmax(p->p[0]),
		ymin(p->p[1]), ymax(p->p[1]),
		zmin(p->p[2]), zmax(p->p[2])
{}

PointcloudProcessor::BoundingBox::BoundingBox()
:
		img_xmin(0), img_xmax(0),
		img_ymin(0), img_ymax(0),
		xmin(0), xmax(0),
		ymin(0), ymax(0),
		zmin(0), zmax(0)
{}

void PointcloudProcessor::BoundingBox::setToPoint(int16_t x, int16_t y,
		const point_t* p) {
	img_xmin=x; img_xmax=x;
	img_ymin=y; img_ymax=y;
	xmin=p->p[0]; xmax=p->p[0];
	ymin=p->p[1]; ymax=p->p[1];
	zmin=p->p[2]; zmax=p->p[2];
}

bool PointcloudProcessor::BoundingBox::intersects(const BoundingBox& b) {
	if(b.xmax<xmin || b.xmin>xmax) return false;
	if(b.ymax<ymin || b.ymin>ymax) return false;
	if(b.zmax<zmin || b.zmin>zmax) return false;
	return true;
}

void PointcloudProcessor::ColorBbox::set(double camHeight,
		const openni::VideoStream &d, const openni::VideoStream &c,
		const BoundingBox& bbox, const Eigen::Matrix3d cam2robot) {
	Eigen::Matrix3d robot2cam=cam2robot.inverse();
	Eigen::Vector3d corner;
	corner=robot2cam*Eigen::Vector3d(bbox.xmax,bbox.ymax,bbox.zmax-camHeight);
	openni::CoordinateConverter::convertDepthToColor(
			d,c,
			bbox.img_xmin,bbox.img_ymin,lrint((double)corner[2]),
			&x1, &y1);
	corner=robot2cam*Eigen::Vector3d(bbox.xmin,bbox.ymin,bbox.zmin-camHeight);
	openni::CoordinateConverter::convertDepthToColor(
			d,c,
			bbox.img_xmax,bbox.img_ymax,lrint((double)corner[2]),
			&x2, &y2);
	std::cerr
		<<(bbox.xmax-bbox.xmin)<<'x'
		<<(bbox.ymax-bbox.ymin)<<'x'
		<<(bbox.zmax-bbox.zmin)<<"; "
		<<x1<<'|'<<y1<<" : "
		<<x2<<'|'<<y2<<" : "<<'\n';
	x1-=80;
	y1-=30;
	x2+=10;
	y2+=10;
	int cw=c.getVideoMode().getResolutionX();
	int ch=c.getVideoMode().getResolutionY();
	if(x1<0) x1=0;
	if(y1<0) y1=0;
	if(x2>=cw) x2=cw-1;
	if(y2>=ch) y2=ch-1;
}

bool PointcloudProcessor::ColorBbox::isInside(int x, int y) {
	if(x<x1 || x>x2) return false;
	if(y<y1 || y>y2) return false;
	return true;
}
