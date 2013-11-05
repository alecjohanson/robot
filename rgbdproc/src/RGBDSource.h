/**
 * @file RGBDSource.h
 * Description.
 * @author Christian Thießen
 * @copyright &copy; 2013 Christian Thießen
 * @date Oct 7, 2013
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

#ifndef RGBDSOURCE_H_
#define RGBDSOURCE_H_

#include <OpenNI.h>
#include <stdexcept>
#include <set>

class RGBDSource {
public:
	RGBDSource(const char *uri);
	~RGBDSource();
	void setVideoMode(const openni::SensorType stype, const int w, const int h,
			const int fps, openni::PixelFormat pixfmt)
			throw(std::runtime_error);
	openni::VideoMode getVideoMode(const openni::SensorType stype);
	openni::VideoStream &getVideoStream(const openni::SensorType stype);
	void startStreams();
	void stopStreams();
private:
//	RGBDSource(RGBDSource const &copy);
//	RGBDSource &operator=(RGBDSource const& copy);
	void lazyInit(const openni::SensorType stype);
	openni::Device m_dev;
	openni::VideoStream m_vstreams[3];
};

#endif /* RGBDSOURCE_H_ */
