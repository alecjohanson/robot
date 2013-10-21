/*
 * SVGParser.h
 *
 *  Created on: Oct 20, 2013
 *      Author: robo
 */

#ifndef SVGPARSER_H_
#define SVGPARSER_H_

#include <vector>
#include <xercesc/sax2/DefaultHandler.hpp>
#include "Wall.h"

class SVGParser: public xercesc::DefaultHandler {
public:
	SVGParser(std::vector<Wall> &walls);
	virtual ~SVGParser();

	//SAX2 XML interface
    virtual void startDocument();
    virtual void endDocument();
    virtual void startElement
    (
        const   XMLCh* const    uri,
        const   XMLCh* const    localname,
        const   XMLCh* const    qname,
        const xercesc::Attributes&	attrs
    );
    virtual void endElement
	(
		const XMLCh* const uri,
		const XMLCh* const localname,
		const XMLCh* const qname
	);
private:
	int m_xml_ignore;
	double m_translate_x, m_translate_y;
	double m_width, m_height;
	std::vector<Wall> &m_walls;
	static const double SVG_UU_TO_METERS=25.4e-3/90.0;

	double getDoubleArg(const xercesc::Attributes&	attrs, const char *name);
    char * getStringArg(const xercesc::Attributes&	attrs, const char *name);

};

#endif /* SVGPARSER_H_ */
