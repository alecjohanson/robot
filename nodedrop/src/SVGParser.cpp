/*
 * SVGParser.cpp
 *
 *  Created on: Oct 20, 2013
 *      Author: robo
 */

#include "SVGParser.h"
#include <cmath>
#include <xercesc/sax2/Attributes.hpp>

SVGParser::SVGParser(std::vector<Wall> &walls):
	m_xml_ignore(0),
	m_translate_x(0.0),
	m_translate_y(0.0),
	m_width(0.0),
	m_height(0.0),
	m_walls(walls)
{
}

SVGParser::~SVGParser() {
}


void SVGParser::startDocument() {
	m_translate_x=0.0;
	m_translate_y=0.0;
	m_walls.clear();
}

void SVGParser::endDocument() {
}

void SVGParser::startElement(const XMLCh* const uri, const XMLCh* const localname,
		const XMLCh* const qname, const xercesc::Attributes& attrs) {

	char *elname=xercesc::XMLString::transcode(qname);
	//std::cerr<<'<'<<elname<<'>'<<std::endl;

	if(m_xml_ignore>0) {
		++m_xml_ignore;

	} else if(!strcmp(elname,"svg")) {
		m_width=getDoubleArg(attrs,"width")*SVG_UU_TO_METERS;
		m_height=getDoubleArg(attrs,"height")*SVG_UU_TO_METERS;

	} else if(!strcmp(elname,"g")) {
		char *transform=getStringArg(attrs,"transform");
		char *p1=strchr(transform,'(');
		if(p1 && !strncmp(transform,"translate",p1-transform)) {
			++p1;
			char *p2;
			m_translate_x+=strtod(p1,&p2);
			p1=p2+1;
			m_translate_y+=strtod(p1,&p2);
		}
		xercesc::XMLString::release(&transform);
	} else if(!strcmp(elname,"line")) {
		double x1=(getDoubleArg(attrs,"x1")+m_translate_x)*SVG_UU_TO_METERS;
		double y1=(getDoubleArg(attrs,"y1")+m_translate_y)*SVG_UU_TO_METERS;
		double x2=(getDoubleArg(attrs,"x2")+m_translate_x)*SVG_UU_TO_METERS;
		double y2=(getDoubleArg(attrs,"y2")+m_translate_y)*SVG_UU_TO_METERS;
		char *style=getStringArg(attrs,"style");
		double t=20e-3;
		char *p1=strstr(style,"stroke-width:");
		if(p1) {
			char *p2;
			p1+=13;
			double _t=strtod(p1,&p2);
			if(p2>p1) t=_t*SVG_UU_TO_METERS;
		}
		xercesc::XMLString::release(&style);
		Wall wl=Wall(Vec2D<double>(x1,y1),Vec2D<double>(x2,y2),t);
		m_walls.push_back(wl);

	} else if(!strcmp(elname,"rect")) {
		double x=(getDoubleArg(attrs,"x")+m_translate_x)*SVG_UU_TO_METERS;
		double y=(getDoubleArg(attrs,"y")+m_translate_y)*SVG_UU_TO_METERS;
		double w=getDoubleArg(attrs,"width")*SVG_UU_TO_METERS;
		double h=getDoubleArg(attrs,"height")*SVG_UU_TO_METERS;
		char *style=getStringArg(attrs,"style");
		double t=20e-3;
		char *p1=strstr(style,"stroke-width:");
		if(p1) {
			char *p2;
			p1+=13;
			double _t=strtod(p1,&p2);
			if(p2>p1) t=_t*SVG_UU_TO_METERS;
		}
		xercesc::XMLString::release(&style);
		Wall wl=Wall(Vec2D<double>(x,y),Vec2D<double>(x+w,y),t);
		m_walls.push_back(wl);
		wl=Wall(Vec2D<double>(x,y),Vec2D<double>(x,y+h),t);
		m_walls.push_back(wl);
		wl=Wall(Vec2D<double>(x+w,y),Vec2D<double>(x+w,y+h),t);
		m_walls.push_back(wl);
		wl=Wall(Vec2D<double>(x,y+h),Vec2D<double>(x+w,y+h),t);
		m_walls.push_back(wl);

	} else {
		++m_xml_ignore;
	}

	xercesc::XMLString::release(&elname);
}

void SVGParser::endElement(const XMLCh* const uri, const XMLCh* const localname,
		const XMLCh* const qname) {
	if(m_xml_ignore>0) --m_xml_ignore;
}

double SVGParser::getDoubleArg(const xercesc::Attributes& attrs, const char* name) {
	XMLCh* xName=xercesc::XMLString::transcode(name);
	const XMLCh* xValue=attrs.getValue(xName);
	xercesc::XMLString::release(&xName);
	if(!xValue) return NAN;
	char *value=xercesc::XMLString::transcode(xValue);
	char *p=value;
	double r=strtod(value,&p);
	if(p==value) r=NAN;
	xercesc::XMLString::release(&value);
	return r;
}

char * SVGParser::getStringArg(const xercesc::Attributes& attrs, const char* name) {
	XMLCh* xName=xercesc::XMLString::transcode(name);
	const XMLCh* xValue=attrs.getValue(xName);
	xercesc::XMLString::release(&xName);
	if(!xValue) return 0;
	return xercesc::XMLString::transcode(xValue);
}
