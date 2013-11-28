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
#include "math.h"

// IR offsets in robot frame
const Vec2D<double> IR0_off(0.06,-0.05);
const Vec2D<double> IR1_off(0.07,0.0);
const Vec2D<double> IR2_off(0.06,0.05);
const Vec2D<double> IR3_off(-0.06,0.05);
const Vec2D<double> IR4_off(-0.07,0.0);
const Vec2D<double> IR5_off(-0.06,-0.05);

/*const Vec2D<double> IR0_off(0,0);
const Vec2D<double> IR1_off(0,0);
const Vec2D<double> IR2_off(0,0);
const Vec2D<double> IR3_off(0,0);
const Vec2D<double> IR4_off(0,0);
const Vec2D<double> IR5_off(0,0);
*/

void Maze::drawRobot(Vec2D<double> pos, Vec2D<double> dir) {
	// normalize direction vector
	double L = pow(pow(dir.x,2)+pow(dir.y,2),0.5);
	dir.x /= L; 
	dir.y /= L;
	double theta = atan2(dir.y,dir.x);

	// draw robot body
	visualization_msgs::Marker mark;
	mark.header.frame_id="/sim_map";
	mark.header.stamp=ros::Time();
	mark.ns="robot";
	mark.action = visualization_msgs::Marker::ADD;
	mark.id=0;
	mark.type=visualization_msgs::Marker::SPHERE;
	mark.pose.position.x = pos.x;
	mark.pose.position.y = pos.y;
	mark.pose.position.z = 0;
	// quaternions
	mark.pose.orientation.w = 1.0; // identity
	mark.scale.x = 0.05;
	mark.scale.y = 0.05;
	mark.scale.z = 0.05;
	mark.color.r = 1.0f;
	mark.color.g = 0.0f;
	mark.color.b = 0.0f;
	mark.color.a = 1.0;
	mark.lifetime = ros::Duration();
	m_marker_pub.publish(mark);


	// draw IRs
	// quaternions
	mark.pose.orientation.w = 1.0; // identity
	mark.scale.x = 0.02;
	mark.scale.y = 0.02;
	mark.scale.z = 0.02;
	mark.color.r = 0.0f;
	mark.color.g = 1.0f;
	mark.color.b = 0.0f;
	mark.color.a = 1.0;
	mark.header.frame_id="/sim_map";
	mark.header.stamp=ros::Time();
	mark.ns="robot";
	mark.action = visualization_msgs::Marker::ADD;
	mark.lifetime = ros::Duration();
	mark.type=visualization_msgs::Marker::ARROW;
	mark.pose.position.z = 0;
	mark.pose.position.x = 0;
	mark.pose.position.y = 0;

	// 0
	mark.id=1;
	Vec2D<double> temp(pos.x + IR0_off.x*cos(theta)-IR0_off.y*sin(theta),pos.y + IR0_off.x*sin(theta)+IR0_off.y*cos(theta));
	mark.points.push_back(temp.toPoint());
	temp.x = pos.x + (IR0_off.x)*cos(theta)-(IR0_off.y-0.02)*sin(theta);
	temp.y = pos.y + (IR0_off.x)*sin(theta)+(IR0_off.y-0.02)*cos(theta);
	mark.points.push_back(temp.toPoint());	
	m_marker_pub.publish(mark);
	// 1, FRONT
	mark.color.r = 1.0f;
	mark.color.g = 0.0f;
	mark.id=2;
	temp.x = pos.x + (IR1_off.x)*cos(theta)-(IR1_off.y)*sin(theta);
	temp.y = pos.y + (IR1_off.x)*sin(theta)+(IR1_off.y)*cos(theta);
	mark.points[0] = temp.toPoint();
	temp.x = pos.x + (IR1_off.x+0.02)*cos(theta)-(IR1_off.y)*sin(theta);
	temp.y = pos.y + (IR1_off.x+0.02)*sin(theta)+(IR1_off.y)*cos(theta);
	mark.points[1] = temp.toPoint();
	m_marker_pub.publish(mark);
	// 2
	mark.color.r = 0.0f;
	mark.color.g = 1.0f;
	mark.id=3;
	temp.x = pos.x + (IR2_off.x)*cos(theta)-(IR2_off.y)*sin(theta);
	temp.y = pos.y + (IR2_off.x)*sin(theta)+(IR2_off.y)*cos(theta);
	mark.points[0] = temp.toPoint();
	temp.x = pos.x + (IR2_off.x)*cos(theta)-(IR2_off.y+0.02)*sin(theta);
	temp.y = pos.y + (IR2_off.x)*sin(theta)+(IR2_off.y+0.02)*cos(theta);
	mark.points[1] = temp.toPoint();
	m_marker_pub.publish(mark);
	// 3
	mark.id=4;
	temp.x = pos.x + (IR3_off.x)*cos(theta)-(IR3_off.y)*sin(theta);
	temp.y = pos.y + (IR3_off.x)*sin(theta)+(IR3_off.y)*cos(theta);
	mark.points[0] = temp.toPoint();
	temp.x = pos.x + (IR3_off.x)*cos(theta)-(IR3_off.y+0.02)*sin(theta);
	temp.y = pos.y + (IR3_off.x)*sin(theta)+(IR3_off.y+0.02)*cos(theta);
	mark.points[1] = temp.toPoint();
	m_marker_pub.publish(mark);
	// 4
	mark.id=5;
	temp.x = pos.x + (IR4_off.x)*cos(theta)-(IR4_off.y)*sin(theta);
	temp.y = pos.y + (IR4_off.x)*sin(theta)+(IR4_off.y)*cos(theta);
	mark.points[0] = temp.toPoint();
	temp.x = pos.x + (IR4_off.x-0.02)*cos(theta)-(IR4_off.y)*sin(theta);
	temp.y = pos.y + (IR4_off.x-0.02)*sin(theta)+(IR4_off.y)*cos(theta);
	mark.points[1] = temp.toPoint();
	m_marker_pub.publish(mark);
	// 5
	mark.id=6;
	temp.x = pos.x + (IR5_off.x)*cos(theta)-(IR5_off.y)*sin(theta);
	temp.y = pos.y + (IR5_off.x)*sin(theta)+(IR5_off.y)*cos(theta);
	mark.points[0] = temp.toPoint();
	temp.x = pos.x + (IR5_off.x)*cos(theta)-(IR5_off.y-0.02)*sin(theta);
	temp.y = pos.y + (IR5_off.x)*sin(theta)+(IR5_off.y-0.02)*cos(theta);
	mark.points[1] = temp.toPoint();
	m_marker_pub.publish(mark);

}



Maze::Maze():
	m_walls(),
	m_width(0.0),
	m_height(0.0)
{
	ros::NodeHandle n;
	std::cerr<<"\tAdvertising maze marker topic to ROS\n";
	m_marker_pub=n.advertise<visualization_msgs::Marker>("maze_markers",10);
	// wait until rviz has subscribed to topic, otherwise messages will be lost
	std::cerr << "\tWaiting for RVIZ to connect...";
	while (m_marker_pub.getNumSubscribers () == 0 ){
 	}
	std::cerr << " OK" << std::endl;
}

Maze::~Maze() {
}

// SIMULATE THE SHARPS WITH THE LOADED MAP
std::vector<double> Maze::simulateSharps(Vec2D<double> pos,Vec2D<double> dir){
// pose is x,y, and angle. absolute flipped with respect to rviz coordinates
	std::vector<double> result (6,0);
	// iterate through all the walls in the maze
	Vec2D<double> p;

	// normalize direction vector
	double L = pow(pow(dir.x,2)+pow(dir.y,2),0.5);
	dir.x /= -L; // flippin axis
	dir.y /= -L;
	double theta = atan2(dir.y,dir.x);
	double IR0 = INFINITY;
	double IR1 = INFINITY;
	double IR2 = INFINITY;
	double IR3 = INFINITY;
	double IR4 = INFINITY;
	double IR5 = INFINITY;


	Vec2D<double> temp(0,0);
	for (int i=0;i<m_walls.size();++i){
	// check 1 front sensor
		temp.x = pos.x - IR1_off.x*cos(theta)-IR1_off.y*sin(theta);
		temp.y = pos.y - IR1_off.x*sin(theta)+IR1_off.y*cos(theta);
		if(IR1 > m_walls[i].getDistance(temp,dir)) IR1 = m_walls[i].getDistance(temp,dir);
	// check 2, g
		dir = dir.rot90L();
		temp.x = pos.x - IR2_off.x*cos(theta)-IR2_off.y*sin(theta);
		temp.y = pos.y - IR2_off.x*sin(theta)+IR2_off.y*cos(theta);
		if(IR2 > m_walls[i].getDistance(temp,dir)) IR2 = m_walls[i].getDistance(temp,dir);
	// check 3
		temp.x = pos.x - IR3_off.x*cos(theta)-IR3_off.y*sin(theta);
		temp.y = pos.y - IR3_off.x*sin(theta)+IR3_off.y*cos(theta);
		if(IR3 > m_walls[i].getDistance(temp,dir)) IR3 = m_walls[i].getDistance(temp,dir);
	// check 4  back
		dir = dir.rot90L();
		temp.x = pos.x - IR4_off.x*cos(theta)-IR4_off.y*sin(theta);
		temp.y = pos.y - IR4_off.x*sin(theta)+IR4_off.y*cos(theta);
		if(IR4 > m_walls[i].getDistance(temp,dir)) IR4 = m_walls[i].getDistance(temp,dir);
	// check 5
		dir = dir.rot90L();
		temp.x = pos.x - IR5_off.x*cos(theta)-IR5_off.y*sin(theta);
		temp.y = pos.y - IR5_off.x*sin(theta)+IR5_off.y*cos(theta);
		if(IR5 > m_walls[i].getDistance(temp,dir)) IR5 = m_walls[i].getDistance(temp,dir);
	// check 0
		temp.x = pos.x - IR0_off.x*cos(theta)-IR0_off.y*sin(theta);
		temp.y = pos.y - IR0_off.x*sin(theta)+IR0_off.y*cos(theta);
		if(IR0 > m_walls[i].getDistance(temp,dir)) IR0 = m_walls[i].getDistance(temp,dir);
		dir = dir.rot90L();
	}




	result[0] = IR0;
	result[1] = IR1;
	result[2] = IR2;
	result[3] = IR3;
	result[4] = IR4;
	result[5] = IR5;


return result;
}


void Maze::removeNode(int id) {
// Remove Sphere
	visualization_msgs::Marker node;
	node.header.frame_id="/sim_map";
	node.header.stamp=ros::Time();
	node.ns="nodes";
	node.action = visualization_msgs::Marker::DELETE;
	node.id=id;
	m_marker_pub.publish(node);
// remove arrows
	id = id*4;
	for(int i=0;i<4;++i) {
		visualization_msgs::Marker node;
		node.header.frame_id="/sim_map";
		node.header.stamp=ros::Time();
		node.ns="arrows";
		node.action = visualization_msgs::Marker::DELETE;
		node.id=id;
		id++;
		m_marker_pub.publish(node);
	}
}


void Maze::drawNode(MapNode* n_node) {
	int id = n_node->getID();
	// draw the sphere
	visualization_msgs::Marker node;
	node.header.frame_id="/sim_map";
	node.header.stamp=ros::Time();
	node.ns="nodes";
	node.action = visualization_msgs::Marker::ADD;
	node.id=id;
	node.type=visualization_msgs::Marker::CUBE;
	node.pose.position.x = n_node->getPos().x;
	node.pose.position.y = n_node->getPos().y;
	node.pose.position.z = 0;
	// quaternions
	node.pose.orientation.w = 1.0; // identity
	node.scale.x = 0.03;
	node.scale.y = 0.03;
	node.scale.z = 0.03;
	node.color.r = 0.0f;
	node.color.g = 1.0f;
	node.color.b = 0.0f;
	node.color.a = 1.0;
	node.lifetime = ros::Duration();
	m_marker_pub.publish(node);

	//draw arrows
	id = id*4;
	for(int i=0;i<4;++i) {
		if (n_node->getExplored(i)>0) {	
			visualization_msgs::Marker arrow;
			arrow.header.frame_id="/sim_map";
			arrow.header.stamp=ros::Time();
			arrow.ns="arrows";
			arrow.action = visualization_msgs::Marker::ADD;
			arrow.id=id;
			++id;
			arrow.type=visualization_msgs::Marker::ARROW;
			arrow.pose.position.x = 0;//n_node->GetPos().x;
			arrow.pose.position.y = 0;//n_node->GetPos().y;
			arrow.pose.position.z = 0;
			arrow.pose.orientation.w = 1.0; // identity
			// orientation
			Vec2D<double> temp; double l = 0.07; // length of arrow
			temp = n_node->getPos();
				switch (i) {
					case 0:
						temp.x += -l; temp.y += 0;
					break;
					case 1:
						temp.x += 0; temp.y += -l;
					break;
					case 2:
						temp.x += l; temp.y += 0;
					break;
					case 3:
						temp.x += 0; temp.y += l;
					break;
				}
			arrow.points.push_back(n_node->getPos().toPoint());
			arrow.points.push_back(temp.toPoint());

			arrow.scale.x = 0.01;
			arrow.scale.y = 0.01;
			arrow.scale.z = 0.01;
			arrow.color.r = 1.0f;
			arrow.color.g = 0.0f;
			arrow.color.b = 0.0f;
			arrow.color.a = 1.0;
			if (n_node->getExplored(i) == 2) {// explored node
				arrow.color.r = 0.5f;
				arrow.color.g = 0.5f;
				arrow.color.b = 0.5f;
				arrow.color.a = 1;	
			}
			arrow.lifetime = ros::Duration();
			m_marker_pub.publish(arrow);
		}
	}
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
	std::cerr<<m_walls.size()<<" Walls found. Publishing Maze... ";
    publishMaze();
	std::cerr<<"OK\n";
}

void Maze::publishMaze() const {
	visualization_msgs::Marker line_list;
	line_list.header.frame_id="/sim_map";
	line_list.header.stamp=ros::Time();
	line_list.ns="SimMaze";
	line_list.id=0;
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

