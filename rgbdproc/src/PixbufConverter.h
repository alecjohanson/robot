/**
 * @file PixbufConverter.h
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

#ifndef PIXBUFCONVERTER_H_
#define PIXBUFCONVERTER_H_

#include <OpenNI.h>
#include <gdk-pixbuf/gdk-pixbuf-core.h>
#include <gtk/gtk.h>

class PixbufConverter: public openni::VideoStream::NewFrameListener {
public:
	PixbufConverter(const openni::VideoMode &m);
	~PixbufConverter();
	void setTarget(GtkWidget *target);
protected:
	virtual void onNewFrame(openni::VideoStream& v);
private:
	int w,h;
	GdkPixbuf *m_pixbuf;
	GtkWidget *m_target;
	gboolean m_prev_app_paintable, m_prev_double_buffered;
	gint m_prev_sizerq_w, m_prev_sizerq_h;
	gulong m_sighandler;
	static const unsigned int DIST_MIN=3000, DIST_MAX=14000;
};

#endif /* PIXBUFCONVERTER_H_ */
