/**
 * @file RGBDSource.cpp
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

#include "RGBDSource.h"
#include <iostream>

RGBDSource& RGBDSource::getInstance() {
	static RGBDSource instance;
	return instance;
}

void RGBDSource::setVideoMode(openni::SensorType stype, int w, int h, int fps,
		openni::PixelFormat pixfmt) throw(std::runtime_error) {
	openni::VideoMode v;
	v.setResolution(w,h);
	v.setFps(30);
	v.setPixelFormat(pixfmt);
	lazyInit(stype);

	openni::Status s=m_vstreams[stype-1].setVideoMode(v);
	if(s!=openni::STATUS_OK)
		throw std::runtime_error(openni::OpenNI::getExtendedError());
}

openni::VideoMode RGBDSource::getVideoMode(
		const openni::SensorType stype) {
	lazyInit(stype);
	return m_vstreams[stype-1].getVideoMode();
}

openni::VideoStream& RGBDSource::getVideoStream(
		const openni::SensorType stype) {
	lazyInit(stype);
	return m_vstreams[stype-1];
}

void RGBDSource::startStreams() {
	for(int i=0; i<3;++i)
		if(m_vstreams[i].isValid()) m_vstreams[i].start();
}

void RGBDSource::stopStreams() {
	for(int i=0; i<3;++i)
		if(m_vstreams[i].isValid()) m_vstreams[i].stop();
}

RGBDSource::RGBDSource() {
	openni::Status s;
	s=openni::OpenNI::initialize();
	if(s!=openni::STATUS_OK) {
		std::cerr << "OpenNI Error: " << openni::OpenNI::getExtendedError()
			<< std::endl;
		exit(-1);
	}
	s=m_dev.open(openni::ANY_DEVICE);
	if(s!=openni::STATUS_OK) {
		std::cerr << "OpenNI Error: " << openni::OpenNI::getExtendedError()
			<< std::endl;
		exit(-1);
	}
	m_dev.setDepthColorSyncEnabled(true);
	m_dev.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
}

RGBDSource::~RGBDSource() {
	for(int i=0; i<3;++i) {
		if(m_vstreams[i].isValid()) m_vstreams[i].destroy();
	}
	openni::OpenNI::shutdown();
}

void RGBDSource::lazyInit(const openni::SensorType stype) {
	openni::Status s;
	if(!m_vstreams[stype-1].isValid()) {
		s=m_vstreams[stype-1].create(m_dev,stype);
		if(s!=openni::STATUS_OK)
			throw std::runtime_error(openni::OpenNI::getExtendedError());
	}
}
