/*
 * mapserver.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: robo
 */

#include <iostream>
#include <xercesc/util/PlatformUtils.hpp>
#include <ros/ros.h>
#include "Wall.h"
#include "Maze.h"

int main(int argc, char **argv) {
	std::cerr<<"Initializing XML\n";
	try {
		xercesc::XMLPlatformUtils::Initialize();
	}
	catch (const xercesc::XMLException& toCatch) {
		char* message = xercesc::XMLString::transcode(toCatch.getMessage());
		std::cout << "Error during initialization! :\n";
		std::cout << "Exception message is: \n"
				<< message << "\n";
		xercesc::XMLString::release(&message);
		return 1;
	}
	std::cerr<<"Initializing ROS\n";
	ros::init(argc,argv,"mapserver");
	std::cerr<<"Creating maze object\n";
	Maze m;
	std::cerr<<"Loading maze from SVG\n";
	/* TODO: Locate files relative to package directory?
	 * #include <ros/package.h>
	 * std::string path = ros::package::getPath(ros::this_node::getName());
	 */
	m.loadFromSVG("testdata/maze001.svg");
	std::cerr<<"Done.\n";
	ros::spin();
	return 0;
}
