/*
 * Maze.h
 *
 *  Created on: Oct 17, 2013
 *      Author: robo
 */

#ifndef MAZE_H_
#define MAZE_H_

#include <string>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <ros/ros.h>
#include "mapserver/fake_camera.h"
#include "mapserver/fake_sharp.h"
#include "Wall.h"

class Maze {
public:
	Maze();
	virtual ~Maze();

	/*
	 * TODO: Methods to receive true robot location, camera orientation.
	 */
	bool simulateCamera(mapserver::fake_cameraRequest& request, mapserver::fake_cameraResponse& response);
	bool simulateSharp(mapserver::fake_sharpRequest& request, mapserver::fake_sharpResponse& response);

	void loadFromSVG(const std::string &fname);
private:
	std::vector<Wall> m_walls;
	double m_width, m_height;
	ros::Publisher m_marker_pub;

	void publishMaze() const;
};

#endif /* MAZE_H_ */
