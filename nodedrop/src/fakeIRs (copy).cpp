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
#include "Vec2D.h"
#include <visualization_msgs/Marker.h>
#include "MapNode.h"


const int r_left = 1;
const int r_right = 3;
const int r_front = 0;
const int r_back = 2;

// IR offsets in robot frame
const Vec2D<double> IR0_off(0.06,-0.05);
const Vec2D<double> IR1_off(0.07,0.0);
const Vec2D<double> IR2_off(0.06,0.05);
const Vec2D<double> IR3_off(-0.06,0.05);
const Vec2D<double> IR4_off(-0.07,0.0);
const Vec2D<double> IR5_off(-0.06,-0.05);

MapNode * nearNode(MapNode * node, Vec2D<double> pos,double bsize) {
	// loop through all nodes
	while (node != NULL) {
		double x = node->getPos().x;
		double y = node->getPos().y;
		if ( fabs(x-pos.x) < 0.5*bsize && fabs(y-pos.y) < 0.5*bsize) {
			return node;
		}
		node = node->getChild();
	}
	
	return NULL;
}


int dir_robot2frame(int i,Vec2D<double> dir) {
	int tdir;	
	if (dir.x==-1) tdir = 0;
	if (dir.x== 1) tdir = 2;
	if (dir.y==-1) tdir = 1;
	if (dir.y== 1) tdir = 3;
	return (tdir+i)% 4;
}


std::vector<int> createNodeDir(std::vector<double> IR,Vec2D<double> dir,double& IR_limit) {
	std::vector<int> n_dir (4,0);	
	if ((IR[2] > IR_limit) && (IR[3] > IR_limit)) n_dir[dir_robot2frame(r_left, dir)] = 1;			
	if (IR[1] > IR_limit) n_dir[dir_robot2frame(r_front, dir)] = 1;			
	if (IR[4] > IR_limit) n_dir[dir_robot2frame(r_back, dir)] = 1;			
	if ((IR[0] > IR_limit) && (IR[5] > IR_limit)) n_dir[dir_robot2frame(r_right, dir)] = 1;	
	return n_dir;
}




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
	ros::NodeHandle n;

	std::cerr<<"Loading maze from SVG\n";
	/* TODO: Locate files relative to package directory?
	 * #include <ros/package.h>
	 * std::string path = ros::package::getPath(ros::this_node::getName());
	 */
	m.loadFromSVG("testdata/maze001.svg");



	Vec2D<double> pos;
	Vec2D<double> dir;
	pos.x = 0.6; pos.y = 0.15; 
	dir.x = 0; dir.y = 1;  // pointing downwards
	std::vector<double> IR (6,0);
	int tdir;
	int node_id=0;
	int state = 0;
	double step = 0.01;
	double th = 0.1;
	double turns = 0;
	double IR_limit = 0.25;
	std::vector<int> n_dir (4,0);
	double gap_left =0;
	double gap_right=0;

	double gap_size = 0.3;
	double bsize = 0.4;

	// drop initial node
	IR = m.simulateSharps(pos,dir);
	n_dir = createNodeDir(IR,dir,IR_limit);
	MapNode *initnode = new MapNode(pos,n_dir,0);
	initnode->setExplored(0,dir);
	MapNode *currnode = initnode;
	m.drawNode(initnode);
	node_id = 1;
	MapNode * nn;
	ros::Rate r(100);
	MapNode * last_looked_at;
	while (ros::ok) {
		IR = m.simulateSharps(pos,dir);
		m.drawRobot(pos,dir);

		switch (state) {

			case 0: // standby
				state = 1;
			break;
			case 1: // forward
				
				// check if node is near
				nn = nearNode(initnode, pos,0.1);
				if (nn != NULL && nn != last_looked_at) {
					state = 2;
				}


				// check if there is a gap, add a node if it is		
				
				// start gap
				if(gap_right == 0){
					if ((IR[0] > IR_limit)) 
						gap_right = step;}
				else {
					gap_right += fabs(step);
					if ((IR[0] < IR_limit)) // stop gap
						if (gap_right>= gap_size) { // create gap node
							Vec2D<double> tpos(pos.x-dir.x*(0.5*gap_right-fabs(IR0_off.x)), \
									pos.y-dir.y*(gap_right*0.5-fabs(IR0_off.y)));
							IR = m.simulateSharps(tpos,dir);
							n_dir = createNodeDir(IR,dir,IR_limit);	
							// if there is a node in the vicinity, do not create new node
							nn = nearNode(initnode, tpos,bsize);
							if (nn != NULL) {
								std::cerr << "Node Exists" << std::endl;
							}else {
								currnode->setChild(new MapNode(tpos,n_dir,node_id));
								currnode->getChild()->setParent(currnode);
								currnode = currnode->getChild();
								std::cerr << "DROP_RIGHT" << std::endl;
								currnode->setExplored(r_front,dir); // going to
				if (currnode->getParent() != NULL && currnode->getParent()->isExplored()) currnode->setExplored(r_back,dir); // came from
								m.drawNode(currnode);
								node_id++;	
							}
							gap_right = 0;
						}

				}	
				

				if(gap_left == 0){
					if ((IR[2] > IR_limit)) 
						gap_left = step;}			
				else {
					gap_left += fabs(step);
					if ((IR[2] < IR_limit)) // stop gap
						if (gap_left>= gap_size) { // create gap node
							Vec2D<double> tpos(pos.x-dir.x*(gap_left*0.5-fabs(IR2_off.x)), \
									pos.y-dir.y*(gap_left*0.5-fabs(IR2_off.y)));
							IR = m.simulateSharps(tpos,dir);
							n_dir = createNodeDir(IR,dir,IR_limit);	
							// if there is a node in the vicinity, do not create new node
							nn = nearNode(initnode, tpos,bsize);
							if (nn != NULL) {
								std::cerr << "Node Exists" << std::endl;
							}else {
								currnode->setChild(new MapNode(tpos,n_dir,node_id));
								currnode->getChild()->setParent(currnode);
								currnode = currnode->getChild();
								std::cerr << "DROP_LEFT" << std::endl;
								currnode->setExplored(r_front,dir); // going to
								if (currnode->getParent() != NULL && currnode->getParent()->isExplored()) currnode->setExplored(r_back,dir); // came from
								m.drawNode(currnode);
								node_id++;	
							}
							gap_left = 0;
						}

				}	

		


				if(IR[1] > th) {
					pos.x += dir.x*step;
					pos.y += dir.y*step;
				}else{
					gap_left = 0;gap_right = 0;
					state=2;
				}
			break;
			case 2:	// turn
				state = 1; ++turns;				
				n_dir = createNodeDir(IR,dir,IR_limit);			
				// if there is a node in the vicinity, do not create new node
				nn = nearNode(initnode, pos,bsize);
				if (nn != NULL) {
					std::cerr << "Node Exists" << std::endl;


					if (nn->getParent() != NULL && nn->getParent()->isExplored()) nn->setExplored(r_back,dir); // came from
					// choose which way to turn	
					if (nn->getDir()[dir_robot2frame(r_right,dir)] == 1) {
						dir = dir.rot90R(); // turn right		
					}else if (nn->getDir()[dir_robot2frame(r_left,dir)] == 1){
						dir = dir.rot90L(); // turn left			
					}else if (nn->getDir()[dir_robot2frame(r_left,dir)] == 2){
						dir = dir.rot90L(); // turn left
					}else if (nn->getDir()[dir_robot2frame(r_right,dir)] == 2) {
						dir = dir.rot90R(); // turn right
					}else if (nn->getDir()[dir_robot2frame(r_back,dir)] > 0) {
						dir = dir.rot90R().rot90R(); // turn 180
					}
					

					nn->setExplored(r_front,dir); // going to
					last_looked_at = nn;
					m.drawNode(nn);

				}else {				
					currnode->setChild(new MapNode(pos,n_dir,node_id));
					currnode->getChild()->setParent(currnode);
					currnode = currnode->getChild();
					
					if (currnode->getParent() != NULL && currnode->getParent()->isExplored()) currnode->setExplored(r_back,dir); // came from
					// choose which way to turn	
					if (currnode->getDir()[dir_robot2frame(r_right,dir)] > 0) {
						dir = dir.rot90R(); // turn right		
					}else if (currnode->getDir()[dir_robot2frame(r_left,dir)] > 0){
						dir = dir.rot90L(); // turn left			
					}

					currnode->setExplored(r_front,dir); // going to
					last_looked_at = currnode;
					m.drawNode(currnode);
					node_id++;
				}
				if (turns > 100) return 0;

			break;

		}
			ros::spinOnce();
			r.sleep();

	}



return 0;




}
