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
#include "Wall.h"
#include "Vec2D.h"
#include "MapNode.h"

class Maze {
public:
	Maze();
	virtual ~Maze();

	/*
	 * TODO: Methods to receive true robot location, camera orientation.
	 */
	std::vector<double> simulateSharps(Vec2D<double> pos,Vec2D<double> dir);
	void drawNode(MapNode* n_node);
	void removeNode(int id);
	void loadFromSVG(const std::string &fname);
	Wall getWall(const int i) {return m_walls[i];}
	void drawRobot(Vec2D<double> pos, Vec2D<double> dir);
private:
	std::vector<Wall> m_walls;
	double m_width, m_height;
	ros::Publisher m_marker_pub;

	void publishMaze() const;
};

#endif /* MAZE_H_ */
