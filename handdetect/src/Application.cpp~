/**
 * @file main.cpp
 * Contains some interesting coordinate transformation for turning the depth
 * image into a point cloud.
 * This program has been quickly hacked together to see some results. It is a
 * textbook example of horrible code quality: A mess of not-clearly-separated
 * modules exchanges data through global variables and other creative ways.
 * @author Christian Thießen
 * @copyright &copy; 2013 Christian Thießen
 * @date Sep 24, 2013
 */
/*
 *  Copyright 2012 Christian Thießen
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

#ifdef HAVE_GUI
#include <gtk/gtk.h>
	#include "MainWindow.h"
	#include "img_proc/PixbufConverter.h"
	#include "pointcloud_proc/CrossSectionPlotter.h"
#endif
#ifdef HAVE_ROS
	#include <ros/ros.h>
#endif
#include "RGBDSource.h"

#include "img_proc/NearestObjectDetector.h"

//#define RES_X 320
//#define RES_Y 240
#define RES_X 640
#define RES_Y 480

int main(int argc, char *argv[]) {
#ifdef HAVE_ROS
	ros::init(argc, argv, "handdetect");//Creates a node named "FakeMotorsTest"
#endif

#ifdef HAVE_GUI
	//initialize GTK
	if( ! g_thread_supported() ) {
		G_GNUC_BEGIN_IGNORE_DEPRECATIONS;
	    g_thread_init( NULL );
	    G_GNUC_END_IGNORE_DEPRECATIONS;
	}
	gdk_threads_init();
	gdk_threads_enter();
	gtk_init( &argc, &argv );

	MainWindow mainwin;
#endif

	//initialize sensor interface
	RGBDSource &rgbd=RGBDSource::getInstance();
	rgbd.setVideoMode(openni::SENSOR_DEPTH,RES_X,RES_Y,30,
			openni::PIXEL_FORMAT_DEPTH_100_UM);
#ifdef HAVE_GUI
	rgbd.setVideoMode(openni::SENSOR_COLOR,RES_X,RES_Y,30,openni::PIXEL_FORMAT_RGB888);
/*
	PixbufConverter pixconv_color(rgbd.getVideoMode(openni::SENSOR_COLOR));
	rgbd.getVideoStream(openni::SENSOR_COLOR)
			.addNewFrameListener(&pixconv_color);
	pixconv_color.setTarget(mainwin.getGtkImage(0,0));
*/
/*
	PointcloudConverter pconv;
	rgbd.getVideoStream(openni::SENSOR_DEPTH).addNewFrameListener(&pconv);

	WallDetector hdetect(RES_X,RES_Y);
	hdetect.setTarget(mainwin.getGtkImage(1,0));
	pconv.addNewPointcloudListener(&hdetect);
*/
#endif

	NearestObjectDetector nod(RES_X,RES_Y);

#ifdef HAVE_GUI
	nod.setTarget(mainwin.getGtkImage(0,0));
#endif
	rgbd.getVideoStream(openni::SENSOR_DEPTH).addNewFrameListener(&nod);

	rgbd.startStreams();

#ifdef HAVE_ROS
	ros::Rate loop_rate(100);
	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}
#endif

#ifdef HAVE_GUI
	gtk_main();
	gdk_threads_leave();
#endif

	return 0;
}
