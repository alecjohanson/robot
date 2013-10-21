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

#ifndef NEARESTOBJECTDETECTOR_H_
#define NEARESTOBJECTDETECTOR_H_

#include <OpenNI.h>
#ifdef HAVE_GUI
#include <gdk-pixbuf/gdk-pixbuf-core.h>
#include <gtk/gtk.h>
#endif
#ifdef HAVE_ROS
	#include <ros/ros.h>
#endif

class NearestObjectDetector: public openni::VideoStream::NewFrameListener {
public:
	NearestObjectDetector(int w, int h);
	~NearestObjectDetector();
#ifdef HAVE_ROS
	void setROSNode(ros::NodeHandle &n);
#endif
#ifdef HAVE_GUI
	void setTarget(GtkWidget *target);
#endif
protected:
	virtual void onNewFrame(openni::VideoStream& v);
private:
	//physical setup constants
	const double beta;
	const double hcam;
	double sinb, cosb;

	int w,h;
#ifdef HAVE_GUI
	GdkPixbuf *m_pixbuf;
	GtkWidget *m_target;
	gboolean m_prev_app_paintable, m_prev_double_buffered;
	gint m_prev_sizerq_w, m_prev_sizerq_h;
	gulong m_sighandler;
#endif
#ifdef HAVE_ROS
	ros::NodeHandle m_rosnode;
	ros::Publisher m_pointpub;
#endif
	static const double Z_MIN=50.0;
	static const uint16_t DIST_MIN=3000, DIST_MAX=30000;
	static const uint16_t DIST_THRESH=1000;
	uint8_t *m_mask;
};

#endif /* PIXBUFCONVERTER_H_ */
