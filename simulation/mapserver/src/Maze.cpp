/*
 * Maze.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: robo
 */

#include "Maze.h"
#include <visualization_msgs/Marker.h>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#include "SVGParser.h"

Maze::Maze():
		m_walls(),
		m_width(0.0),
		m_height(0.0)
{
	ros::NodeHandle n;
	std::cerr<<"\tAdvertising maze marker topic to ROS\n";
	m_marker_pub=n.advertise<visualization_msgs::Marker>("maze_markers",2,true);
}

Maze::~Maze() {
}

bool Maze::simulateCamera(mapserver::fake_cameraRequest& request,
		mapserver::fake_cameraResponse& response) {
	return true;
}

bool Maze::simulateSharp(mapserver::fake_sharpRequest& request,
		mapserver::fake_sharpResponse& response) {
	return true;
}

void Maze::loadFromSVG(const std::string& fname) {
    xercesc::SAX2XMLReader* parser = xercesc::XMLReaderFactory::createXMLReader();
    parser->setFeature(xercesc::XMLUni::fgSAX2CoreValidation, false);
    parser->setFeature(xercesc::XMLUni::fgSAX2CoreNameSpaces, false);

	std::cerr<<"\tCreating SVGParser\n";
    SVGParser svgp(m_walls);
	std::cerr<<"\tParsing\n";
    parser->setContentHandler(&svgp);
    //parser->setErrorHandler(this);

    try {
        parser->parse(fname.c_str());
    }
    catch (const xercesc::XMLException& toCatch) {
        char* message = xercesc::XMLString::transcode(toCatch.getMessage());
        std::cout << "Exception message is:" << std::endl << message << std::endl;
        xercesc::XMLString::release(&message);
        return;
    }
    catch (const xercesc::SAXParseException& toCatch) {
        char* message = xercesc::XMLString::transcode(toCatch.getMessage());
        std::cout << "Exception message is:" << std::endl << message << std::endl;
        xercesc::XMLString::release(&message);
        return;
    }
    catch (...) {
        std::cout << "Unexpected Exception" << std::endl;
        return;
    }

    delete parser;
	std::cerr<<m_walls.size()<<" Walls found. Publishing Maze\n";
    publishMaze();
}

void Maze::publishMaze() const {
	visualization_msgs::Marker line_list;
	line_list.header.frame_id="/world";
	line_list.header.stamp=ros::Time::now();
	line_list.ns="SimMaze";
	line_list.id=1;
	line_list.type=visualization_msgs::Marker::LINE_LIST;
	line_list.pose.orientation.w = 1.0;
	line_list.scale.x = 20.0e-3;
	if(m_walls.size()>0)
		line_list.scale.x=m_walls[0].getThickness(); //wall thickness: all the same!
	line_list.color.a = 0.5;
	line_list.color.b = 1.0;
	for(std::vector<Wall>::const_iterator w=m_walls.begin();
			w!=m_walls.end(); ++w) {
		line_list.points.push_back(w->getPoint(1.0));
		line_list.points.push_back(w->getPoint(-1.0));
	}
	m_marker_pub.publish(line_list);
}

