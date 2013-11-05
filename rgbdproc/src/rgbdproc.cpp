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
#include "DepthToPointcloudConverter.h"
#include "PixbufConverter.h"

#define RES_X 320
#define RES_Y 240

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
    }

    std::cerr << (f_record?f_record:"-") <<':'<< (f_playback?f_playback:"-") <<std::endl;
	RGBDSource rgbd(f_playback);
	if(!rgbd.getDevice().isFile()) {
		rgbd.setVideoMode(openni::SENSOR_DEPTH,RES_X,RES_Y,30,
				openni::PIXEL_FORMAT_DEPTH_100_UM);
		rgbd.setVideoMode(openni::SENSOR_COLOR,RES_X,RES_Y,30,
				openni::PIXEL_FORMAT_RGB888);
	}
	openni::VideoStream& color=rgbd.getVideoStream(openni::SENSOR_COLOR);
	openni::VideoStream& depth=rgbd.getVideoStream(openni::SENSOR_DEPTH);
	PixbufConverter *pbcolor, *pbdepth;
	if(have_gui) {
		mainwin->setSizeRequest(depth.getVideoMode().getResolutionX(),
				depth.getVideoMode().getResolutionY());
		pbcolor=new PixbufConverter(color.getVideoMode());
		pbcolor->setTarget(mainwin->getGtkImage(0,0));
		color.addNewFrameListener(pbcolor);
		pbdepth=new PixbufConverter(depth.getVideoMode());
		pbdepth->setTarget(mainwin->getGtkImage(1,0));
		depth.addNewFrameListener(pbdepth);
	}
	if(f_record) {
		rec.create(f_record);
		rec.attach(color,true);
		rec.attach(depth,false);
		rec.start();
	}
	DepthToPointcloudConverter pconv;
	pconv.readStreamInfo(depth);
	int lastFrame=0;
	while(ros::ok()){
		openni::VideoFrameRef cframe, dframe;
		color.readFrame(&cframe);
		depth.readFrame(&dframe);
		pconv.onNewFrame(dframe);
		std::cerr << dframe.getFrameIndex()-lastFrame << "; "
				  << dframe.getFrameIndex()-cframe.getFrameIndex() << " / "
				  << dframe.getTimestamp() -cframe.getTimestamp() << std::endl;
		lastFrame=dframe.getFrameIndex();
		cframe.release();
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
