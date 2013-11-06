/**
 * @file MainWindow.h
 * Description.
 * @author Christian Thießen
 * @copyright &copy; 2012 Christian Thießen
 * @date Sep 25, 2013
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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <gtk/gtk.h>

gboolean expose_event_callback(GtkWidget* widget,
		GdkEventExpose* event, gpointer data);

class MainWindow {
public:
	MainWindow();
	virtual ~MainWindow();
	void setSizeRequest(int w, int h);
	GtkWidget *getGtkImage(int x, int y);
private:
	GtkWidget *m_window, *m_grid;
	static const int IMAGES_X=1, IMAGES_Y=1;
public:
	GtkWidget *m_images[IMAGES_Y][IMAGES_X];
};

#endif /* MAINWINDOW_H_ */
