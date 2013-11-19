/**
 * @file MainWindow.cpp
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

#include "MainWindow.h"
#include <gdk/gdk.h>
#include <iostream>
#include <ctime>

gboolean expose_event_callback(GtkWidget* widget,
		GdkEventExpose* event, gpointer data) {
    cairo_t *cr = gdk_cairo_create(gtk_widget_get_window(widget));
    gdk_cairo_region(cr, event->region);
    cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
    GtkAllocation walloc;
    gtk_widget_get_allocation(widget,&walloc);

    gdk_cairo_set_source_pixbuf(cr, static_cast<GdkPixbuf*>(data),
    		walloc.x, walloc.y);

    cairo_rectangle(cr,walloc.x,walloc.y,walloc.width,walloc.height);
    cairo_fill(cr);
    cairo_destroy(cr);

    return TRUE;
}

MainWindow::MainWindow()
: m_window(gtk_window_new(GTK_WINDOW_TOPLEVEL)),
  m_grid(gtk_table_new(IMAGES_Y,IMAGES_X,FALSE))

{
	gtk_window_set_title(GTK_WINDOW(m_window), "RGB-D Viewer");
	g_signal_connect (m_window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

	gtk_table_set_col_spacings(GTK_TABLE(m_grid),0);
	gtk_table_set_row_spacings(GTK_TABLE(m_grid),0);

	for(int y=0; y<IMAGES_Y; ++y) {
		for(int x=0; x<IMAGES_X; ++x) {
			m_images[y][x]=gtk_image_new();
			gtk_table_attach(GTK_TABLE(m_grid),m_images[y][x],x,x+1,y,y+1,
					(GtkAttachOptions)0,(GtkAttachOptions)0,0,0);
		}
	}
	gtk_container_add(GTK_CONTAINER(m_window),m_grid);
	gtk_widget_show_all(m_window);
}

MainWindow::~MainWindow() {
}

void MainWindow::setSizeRequest(int w, int h) {
	for(int x=0; x<IMAGES_X; ++x)
		for(int y=0; y<IMAGES_Y; ++y)
			gtk_widget_set_size_request(m_images[y][x],w,h);
}

GtkWidget* MainWindow::getGtkImage(int x, int y) {
	if(x>=0 && x<IMAGES_X && y>=0 && y<IMAGES_Y)
		return m_images[y][x];
	else
		return 0;
}
