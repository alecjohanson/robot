/*
 * rgbdproc.cpp
 *
 *  Created on: Nov 2, 2013
 *      Author: robo
 */

#include <iostream>
#include <ros/ros.h>
#include "RGBDSource.h"
#include "MainWindow.h"
#include <OpenNI.h>
#include <getopt.h>
#include <gtk/gtk.h>
#include "PointcloudProcessor.h"
#include "PixbufConverter.h"

#define RES_X 320
#define RES_Y 240
#define FPS 30

openni::Recorder rec;

static void exitHandler(void);
MainWindow *mainwin;
int have_gui;

static const struct option long_options[] = {
		{"record",   required_argument,         0, 'r'},
		{"playback", required_argument,         0, 'p'},
		{"gui",            no_argument, &have_gui,   1},
		{0, 0, 0, 0}
};

int main(int argc, char **argv) {
	have_gui=0;

	const char *f_record=0, *f_playback=0;
    int option_index = 0;
    while(1) {
    	int c = getopt_long (argc, argv, "abc:d:f:",
    			long_options, &option_index);
    	if(c == -1) break;
    	switch (c) {
    	case 0:
//    		if (long_options[option_index].flag != 0) break;
    		break;
    	case 'r':
    		f_record=optarg;
    		break;
    	case 'p':
    		f_playback=optarg;
    		break;
    	case '?':
    		break;
    	default:
    		abort();
    		break;
    	}
    }
    if(f_record) f_playback=0;
    if(!f_playback) f_playback=openni::ANY_DEVICE;

	atexit(exitHandler);

	ros::init(argc,argv,"rgbdproc");

    if(have_gui) {
    	if(!g_thread_supported()) {
    		G_GNUC_BEGIN_IGNORE_DEPRECATIONS;
    	    g_thread_init(0);
    	    G_GNUC_END_IGNORE_DEPRECATIONS;
    	}
    	gdk_threads_init();
    	gdk_threads_enter();
    	gtk_init(&argc,&argv);
    	mainwin=new MainWindow();
    	gdk_threads_leave();
    }

    std::cerr << (f_record?f_record:"-") <<':'<< (f_playback?f_playback:"-") << ':' <<have_gui <<std::endl;
	RGBDSource rgbd(f_playback);
	if(rgbd.getDevice().isFile()) {
		rgbd.getDevice().getPlaybackControl()->setRepeatEnabled(true);
		rgbd.getDevice().getPlaybackControl()->setSpeed(1.0);
	} else {
		rgbd.setVideoMode(openni::SENSOR_DEPTH,RES_X,RES_Y,30,
				openni::PIXEL_FORMAT_DEPTH_100_UM);
		/*rgbd.setVideoMode(openni::SENSOR_COLOR,RES_X,RES_Y,30,
				openni::PIXEL_FORMAT_RGB888);*/
	}
	//openni::VideoStream& color=rgbd.getVideoStream(openni::SENSOR_COLOR);
	openni::VideoStream& depth=rgbd.getVideoStream(openni::SENSOR_DEPTH);
	PointcloudProcessor pconv;
	pconv.readStreamInfo(depth);
	//PixbufConverter *pbcolor, *pbdepth;
	if(have_gui) {
		pconv.setDisplay(mainwin->getGtkImage(0,0));
		//pbcolor=new PixbufConverter(color.getVideoMode());
		//pbcolor->setTarget(mainwin->getGtkImage(0,0));
		//color.addNewFrameListener(pbcolor);
		//pbdepth=new PixbufConverter(depth.getVideoMode());
		//pbdepth->setTarget(mainwin->getGtkImage(1,0));
		//depth.addNewFrameListener(pbdepth);
	}
	if(f_record) {
		rec.create(f_record);
		//rec.attach(color,false);
		rec.attach(depth,false);
		rec.start();
	}
	rgbd.startStreams();
	bool objectmode=false;
	while(ros::ok()){
		if(have_gui) {
			GDK_THREADS_ENTER();
			while(gtk_events_pending()) gtk_main_iteration();
			GDK_THREADS_LEAVE();
		}
//		openni::VideoFrameRef cframe;
		openni::VideoFrameRef dframe;
		depth.readFrame(&dframe);
//		color.readFrame(&cframe);
		pconv.onNewFrame(dframe);
		if(!rgbd.getDevice().isFile() && pconv.objectDetected()!=objectmode) {
			if(objectmode)
				rgbd.setVideoMode(openni::SENSOR_DEPTH,320,240,30,
								openni::PIXEL_FORMAT_DEPTH_100_UM);
			else
				rgbd.setVideoMode(openni::SENSOR_DEPTH,640,480,30,
								openni::PIXEL_FORMAT_DEPTH_100_UM);
			objectmode=!objectmode;
		}
/*		while((int64_t)dframe.getTimestamp()-(int64_t)cframe.getTimestamp()>500000/FPS) {
			cframe.release();
			color.readFrame(&cframe);
		}
		while((int64_t)cframe.getTimestamp()-(int64_t)dframe.getTimestamp()>500000/FPS) {
			dframe.release();
			depth.readFrame(&dframe);
		std::cerr << (int64_t)dframe.getTimestamp()-(int64_t)cframe.getTimestamp() << std::endl;
		}
		cframe.release();*/
		dframe.release();
		ros::spinOnce();
	}
	return 0;
}

static void exitHandler(void) {
	rec.stop();
	rec.destroy();
	if(mainwin) delete mainwin;
}
