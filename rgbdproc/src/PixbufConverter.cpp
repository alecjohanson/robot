/**
 * @file PixbufConverter.cpp
 * Description.
 * @author Christian Thießen
 * @copyright &copy; 2013 Christian Thießen
 * @date Oct 8, 2013
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

#include "PixbufConverter.h"
#include <iostream>
#include "MainWindow.h"

PixbufConverter::PixbufConverter(const openni::VideoMode &m):
	w(m.getResolutionX()),
	h(m.getResolutionY()),
	m_target(0),
	m_prev_app_paintable(FALSE), m_prev_double_buffered(FALSE),
	m_prev_sizerq_w(0), m_prev_sizerq_h(0),
	m_sighandler(0)
{
	m_pixbuf=gdk_pixbuf_new(GDK_COLORSPACE_RGB,false,8,
			m.getResolutionX(),m.getResolutionY());
}

PixbufConverter::~PixbufConverter() {
	// TODO Auto-generated destructor stub
}

void PixbufConverter::setTarget(GtkWidget* target) {
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

void PixbufConverter::onNewFrame(openni::VideoStream& v) {
	if(!m_target) return;

	openni::VideoFrameRef f;
	v.readFrame(&f);
	int w=f.getWidth();
	int h=f.getHeight();
	uint8_t *dst_img=gdk_pixbuf_get_pixels(m_pixbuf);

	//copy color camera image to a pixbuf
	if(f.getVideoMode().getPixelFormat()==openni::PIXEL_FORMAT_RGB888) {
		const uint8_t *src_img=static_cast<const uint8_t*>(f.getData());
		if(f.getStrideInBytes()==gdk_pixbuf_get_rowstride(m_pixbuf))
			memcpy(dst_img,src_img,f.getDataSize());
		else {
			for(int y=0; y<h;++y)
				memcpy(dst_img+y*gdk_pixbuf_get_rowstride(m_pixbuf),
						src_img+y*f.getStrideInBytes(),
						3*w);
		}

	//convert depth image to RGB grey image in the pixbuf
	} else if(f.getVideoMode().getPixelFormat()
			==openni::PIXEL_FORMAT_DEPTH_100_UM) {
		const uint16_t *src_img=static_cast<const uint16_t*>(f.getData());
		for(int y=0; y<h;++y) {
			const uint16_t *src_line=src_img+(y*f.getStrideInBytes()
					/sizeof(uint16_t));
			uint8_t *dst_val=dst_img+y*gdk_pixbuf_get_rowstride(m_pixbuf);
			for(int x=0; x<w;++x) {
				uint8_t g;
				if(src_line[x]>DIST_MAX) g=0;
				else if(src_line[x]<DIST_MIN) g=255;
				else g=255-((unsigned int)src_line[x]-DIST_MIN)
						*255U/(DIST_MAX-DIST_MIN);
				*dst_val++=g;
				*dst_val++=g;
				*dst_val++=g;
			}
		}
	}
	f.release();
	gdk_threads_enter();
	gtk_widget_queue_draw(m_target);
	gdk_threads_leave();
}
