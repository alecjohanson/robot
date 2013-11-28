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
#include "math.h"
#include <visualization_msgs/Marker.h>
#include "MapNode.h"
#include "../../sharps/src/SensorPublisher.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/Odometry.h>
#include <std_msgs/Int32.h>
#include <string.h>
#include <sharps/Distance.h>
#include <differential_drive/Speed.h>
#include <movement/Movement.h>


static ros::Subscriber sharps_sub;
static ros::Subscriber	enc_sub;
static ros::Subscriber	odo_sub;
static ros::Publisher cmd_pub; //publishes the wanted speed
static ros::Publisher cmd_move; //publishes the wanted move
static ros::Subscriber move_complete_sub; //get "movement complete" messages

const bool debug = true;
const int r_left = 1;
const int r_right = 3;
const int r_front = 0;
const int r_back = 2;
const double w_radius = 0.05;
// IR offsets in robot frame
const Vec2D<double> IR0_off(0.06,-0.05);
const Vec2D<double> IR1_off(0.07,0.0);
const Vec2D<double> IR2_off(0.06,0.05);
const Vec2D<double> IR3_off(-0.06,0.05);
const Vec2D<double> IR4_off(-0.07,0.0);
const Vec2D<double> IR5_off(-0.06,-0.05);

bool ir_received;
bool odometry_received;
double IR_limit = 0.25;
int node_id=0; // keep track of node id's
double d_Wall = 0.1; // distance to keep from wall
double step = 0.01; // step length for simulation [m]
double bsize = 0.2; // size of square which to search for nodes in when dropping new et.c.
double gap_size = 0.2;
double enc_left;
double enc_right;
bool move_complete;
Vec2D<double> pos;
std::vector<double> IR (6,0);
MapNode * initnode = NULL;
std::vector<MapNode*> nodelist;


using namespace differential_drive;

MapNode * nearNode(MapNode * node, Vec2D<double> pos,double bs) {
	// loop through all nodes
	for(int z=0;z<nodelist.size();++z) {
		double x = nodelist[z]->getPos().x;
		double y = nodelist[z]->getPos().y;
		if ( fabs(x-pos.x) < 0.5*bs && fabs(y-pos.y) < 0.5*bs) {
			return nodelist[z];
		}
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


Vec2D<double> dir_frame2robot(int i) {
	Vec2D<double> dir;
	dir.x = dir.y = 0;	
	if (i == 0) dir.x=-1;
	if (i == 2) dir.x= 1;
	if (i == 1) dir.y=-1;
	if (i == 3) dir.y= 1;
	return dir;
}



std::vector<int> createNodeDir(std::vector<double> IR,Vec2D<double> dir) {
	std::vector<int> n_dir (4,0);	
	if ((IR[2] >= IR_limit) && (IR[3] >= IR_limit)) n_dir[dir_robot2frame(r_left, dir)] = 1;			
	if (IR[1] >= IR_limit) n_dir[dir_robot2frame(r_front, dir)] = 1;			
	if (IR[4] >= IR_limit) n_dir[dir_robot2frame(r_back, dir)] = 1;			
	if ((IR[0] >= IR_limit) && (IR[5] >= IR_limit)) n_dir[dir_robot2frame(r_right, dir)] = 1;	
	return n_dir;
}






void connectNode(MapNode *node) {
	// search upward
	// loop through all nodes
	MapNode* nn = initnode;
	double nmin[4] = {INFINITY,INFINITY,INFINITY,INFINITY};
	MapNode* n[4] = {NULL,NULL,NULL,NULL};
	while (nn != NULL) {
		double x = nn->getPos().x;
		double y = nn->getPos().y;
		if ( fabs(x-node->getPos().x) < 0.5*bsize) { // check up-down
			double ty = y-node->getPos().y;			
			if ((ty < 0) && (nn->getExplored(3) > 0) && (node->getExplored(1) > 0) && (fabs(ty) < nmin[1])){n[1] = nn;nmin[1] = fabs(ty);}   // up
			if ((ty > 0) && (nn->getExplored(1) > 0) && (node->getExplored(3) > 0) && (fabs(ty) < nmin[3])){n[3] = nn;nmin[3] = fabs(ty);}   // down
		}
		if ( fabs(y-node->getPos().y) < 0.5*bsize) { // check left-right
			double tx = x-node->getPos().x;			
			if ((tx < 0) && (nn->getExplored(2) > 0) && (node->getExplored(0) > 0) && (fabs(tx) < nmin[0])){n[0] = nn;nmin[0] = fabs(tx);}   // right
			if ((tx > 0) && (nn->getExplored(0) > 0) && (node->getExplored(2) > 0) && (fabs(tx) < nmin[2])){n[2] = nn;nmin[2] = fabs(tx);}   // left
		}
		nn = nn->getNext();
	}
	
	for (int i=0;i<4;++i) {
		if(n[i] != NULL && n[i]->getConnection((i+2)%4) != NULL && n[(i+2)%4] != NULL) {
			// check if adjacent nodes were connected and connect them if they were
			if(n[i]->getConnection((i+2)%4) == n[(i+2)%4]  ) {
				 n[i]->setConnection(node,(i+2) % 4);
				 node->setConnection(n[i],i);
				std::cerr << "fuck";
			}
		}

	}
	

}

MapNode * dropNode(Vec2D<double> pos,Vec2D<double> dir,std::vector<double> IR,MapNode * last){
	MapNode * n = nearNode(initnode,pos,bsize);

	if (n==NULL) {
		if (debug) std::cerr << "ADDED NODE: " << node_id<<std::endl;
		std::vector<int> n_dir (4,0);
		n_dir = createNodeDir(IR,dir); // get available directions

		MapNode * tempnode = new MapNode(pos,n_dir,node_id); //  create node		
		tempnode->setPrev(last);
		connectNode(tempnode); // connect node to grid
		tempnode->setExplored(dir_robot2frame(r_back,dir));
		// save initial node
		if (last == NULL) initnode = tempnode;
		if (last != NULL) {
			last->setNext(tempnode);
			tempnode->setConnection(tempnode->getPrev(),dir_robot2frame(r_back,dir)); // set current node's connection
			tempnode->getPrev()->setConnection(tempnode,dir_robot2frame(r_front,dir)); // set current node's connection
			last->setExplored(dir_robot2frame(r_front,dir));
		}	
		++node_id;
		nodelist.push_back(tempnode);
		return tempnode;
	} else {
		if (debug) std::cerr << "REVISITED NODE: " << n->getID()<<std::endl;
		n->setExplored(dir_robot2frame(r_front,dir));
		return NULL;
	}
}





MapNode * backtrack(MapNode * node) {


	double cur_cost = 0;
	std::vector<MapNode*> frontier;
	std::vector<MapNode*> explored;
	node->setTmp(-1);
	node->setCost(0);
	frontier.push_back(node);
	
	do {
		if (frontier.size() == 0) return NULL; // failure
		int in = 0;
		double mincost = INFINITY;
		// get node with least cost
		for (unsigned int z=0;z<frontier.size();++z)
			if (frontier[z]->getCost() < mincost) {mincost = frontier[z]->getCost(); in = z;}

		std::swap(frontier[in],frontier.back());
		node = frontier.back();
		frontier.pop_back();

		if (node->isExplored() == false) {
			MapNode*temp=node;
			std::cerr << "FOUND a node: " <<node->getID()<< ": COST: " << node->getCost()<< std::endl;
			// reconstruct path
			//std::vector<int> path;
			while(node->getTmp() != -1) {
				//std::cerr <<"node: " <<node->getID()<<"path: " << node->getTmp() << std::endl;
			//	path.push_back(node->getTmp());	
				node->getsPrev()->setsNext(node);		
				node = node->getsPrev();
			}
			//std::reverse(path.begin(),path.end());
			return temp; // solution
		}

		explored.push_back(node); // node checked

		for (int i=0;i<4;++i) {
			int exploredindex = -1;
			int frontierindex = -1;
			MapNode*n = node->getConnection(i);
			if(n != NULL)	{	
				// check if n is in explored set
				for (unsigned int z=0;z<explored.size();++z)
					if (explored[z] == n) {exploredindex = z; break;}
				// check if n is in frontier set
				for (unsigned int z=0;z<frontier.size();++z)
					if (frontier[z] == n) {frontierindex = z; break;}

				if(exploredindex == -1) {
					if (frontierindex == -1) {	
						n->setTmp(i);	
						n->setsPrev(node);	
						n->setCost(node->getCost()+node->getDistance(i));				
						frontier.push_back(n);

			

					} else if ((frontierindex != -1) && (frontier[frontierindex]->getCost()>(node->getCost()+node->getDistance(i)))){
						// if node is in frontier set, but at a higher cost, replace it
						n->setTmp(i);	
						n->setsPrev(node);	
						n->setCost(node->getCost()+node->getDistance(i));	
						frontier[frontierindex] = n;
						std::cerr << "FUCK" << std::endl;
					}
				}
			}
		}
	} while(true);
}

void receive_encoder(const Encoders::ConstPtr &msg)
{
	static ros::Time t_start = ros::Time::now();
	double enc_left = msg->encoder2;
	double enc_right = msg->encoder1;
	enc_right = enc_right/1000 *  2*M_PI*w_radius;
	enc_left = enc_left/1000 *  2*M_PI*w_radius;
	int timestamp = msg->timestamp;
}

void receive_odometry(const Odometry::ConstPtr &msg)
{
	pos.x = msg->x;
	pos.y = msg->y;
	//std::cerr << pos.x << pos.y << std::endl;
	odometry_received = true;

}

void movementCompletedHandler(const movement::Movement &msg) {
	move_complete = true;
}

void receive_ir(const sharps::Distance &msg){
std::vector<double> IR_measurements (6,0);
	IR_measurements[0] = msg.front_r*0.01; 
	IR_measurements[1] = msg.front*0.01; 
	IR_measurements[2] = msg.front_l*0.01; 
	IR_measurements[3] = msg.rear_l*0.01; 
	IR_measurements[4] = msg.rear*0.01; 
	IR_measurements[5] = msg.rear_r*0.01; 
	IR = IR_measurements;
	ir_received = true;
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

	cmd_pub = n.advertise<Speed>("/motion/Speed", 1);
	sharps_sub = n.subscribe("/sharps/Distance/",100,receive_ir);
	enc_sub = n.subscribe("/motion/Encoders", 100, receive_encoder);
	odo_sub = n.subscribe("/motion/Odometry", 100, receive_odometry);

	cmd_move =  n.advertise<movement::Movement>("simpleMovement/move",1);
	move_complete_sub = n.subscribe("simpleMovement/moveCompleted",1,movementCompletedHandler);


	std::cerr<<"Loading maze from SVG\n";
	/* TODO: Locate files relative to package directory?
	 * #include <ros/package.h>
	 * std::string path = ros::package::getPath(ros::this_node::getName());
	 */
	//m.loadFromSVG("testdata/maze001.svg");



	pos.x = 0;
	pos.y = 0;
	Vec2D<double> dir;
	pos.x = 0.6; pos.y = 0.15; 
	dir.x = 0; dir.y = 1;  // pointing downwards
	
	int state = 0;
	std::vector<int> n_dir (4,0);
	MapNode *current_node;
	MapNode *nn;
	
	// set ROS-frequency 
	ros::Rate r(50);
	double turns = 0;
	int last_state = -1;
	double gap_right = 0;
	MapNode * goal;
	bool BACKTRACKING = false;
	bool on_node;
	double last_enc_left = 0;
	double last_enc_right = 0;
	movement::Movement move;
	differential_drive::Speed spd;
	double angle;
	while (ros::ok()) {
		ir_received = false;
		odometry_received = false;

		while (!ir_received && !odometry_received) {
			ros::spinOnce();
			r.sleep();
			if (!ros::ok()) return 0;
		}
		//std::cerr << IR[1] << std::endl;
		//IR = m.simulateSharps(pos,dir);
		m.drawRobot(pos,dir);
		nn = nearNode(initnode, pos,bsize); // find a (if it exists) close node

		on_node = false;
		// see if robot runs over the axis of a already placed node, if  so, set it to current_node
		if ( current_node != nn) {
			if ((fabs(dir.x) == 1)) {  // crossing y-axis
				// check if the nearest nodes y-axis has been crossed
				if (nn != NULL && fabs(nn->getPos().x-pos.x) <= step) {
					current_node = nn; on_node = true;

				}
			} else if ((fabs(dir.y) == 1)) { // crossing x-axis
				if (nn != NULL && fabs(nn->getPos().y-pos.y) <= step) {
					current_node = nn; on_node = true;
				}
			}
		}
		if (on_node==true && BACKTRACKING) state = 4;

		switch (state) {

			case 0: // start up
				// drop initial node
				current_node = dropNode(pos,dir,IR,NULL);
				m.drawNode(current_node);

				// check if robot can move forward, else switch to turn state
				if (current_node->getExplored(dir_robot2frame(r_front,dir)) != 0)
					state = 1; // move forward
				else
					state = 2; // make a turn
			break;
			case 1: // forward
				if (turns > INFINITY) state = 3; // exit
				if (IR[1] < d_Wall) {
					state = 2;
				} else  {

					if(gap_right == 0 ){ // turned off
						if ((IR[0] > IR_limit)) {gap_right = step;}
					}else if(true==false){ //if (BACKTRACKING == false)
						gap_right += fabs(step);
						if (IR[0] < IR_limit){gap_right =0;} // stop gap
						if (gap_right>= gap_size) { // create gap node
							Vec2D<double> tpos(pos.x-dir.x*(0.5*gap_size-fabs(IR0_off.x)), \
									pos.y-dir.y*(gap_size*0.5-fabs(IR0_off.y)));
							//IR = m.simulateSharps(tpos,dir);
							nn = dropNode(tpos,dir,IR,current_node);
							if (nn != NULL) {current_node = nn;}		
							m.drawNode(current_node);
							// draw last one also, could have been updated
							if (current_node->getPrev() != NULL) m.drawNode(current_node->getPrev());  
							
							gap_right = 0;
						}
					}	
					// move robot forward
					//pos.x += dir.x*step;
					//pos.y += dir.y*step;

					spd.W1 = -2.;
					spd.W2 = -2.;
					spd.header.stamp = ros::Time::now();
					cmd_pub.publish(spd);
					

				}

			break;
			case 2:	// turn
				spd.W1 = 0.;
				spd.W2 = 0.;
				spd.header.stamp = ros::Time::now();
				cmd_pub.publish(spd);
				nn = dropNode(pos,dir,IR,current_node);
				gap_right = 0; 
				if (nn != NULL) {current_node = nn;} else if(BACKTRACKING) {current_node = nearNode(initnode, pos,bsize);}
				m.drawNode(current_node);
				// draw last one also, could have been updated
				if (current_node->getPrev() != NULL) m.drawNode(current_node->getPrev());  

				// make a turn
				// choose which way to turn	
				if (current_node->getExplored(dir_robot2frame(r_right,dir)) == 1) {
					dir = dir.rot90R();angle = -90; // turn right		
				}else if (current_node->getExplored(dir_robot2frame(r_left,dir)) == 1){
					dir = dir.rot90L();angle = 90; // turn left			
				}else if (current_node->getExplored(dir_robot2frame(r_back,dir)) == 1){
					dir = dir.rot90L().rot90L(); angle = 180;// turn 180
				}else { // backtrack
					// turn and head to last node
					if(BACKTRACKING == false) {
						BACKTRACKING = true;
						goal = backtrack(current_node);
						if(goal == NULL)  {
							std::cerr<< "MAP EXPLORED!" << std::endl;
							std::cerr << IR[1] << std::endl;
							return 0;}
						dir = dir_frame2robot(current_node->getsNext()->getTmp());
					}
						state = 4; break;
				}
				// turn
					move.turn = true;
					move.magnitude = angle*M_PI/180.0;
					move.header.stamp = ros::Time::now();
					cmd_move.publish(move);
					move_complete = false;
					while(!move_complete) {ros::spinOnce();}
				state = 1; // move forward
				++turns;
			break;
			case 3:
				// loop through all nodes 
				for(int z=0;z<nodelist.size();++z) {
					std::cerr<<"NODE: " << nodelist[z]->getID() << " - Connections" << std::endl;
					for (int i=0;i<4;++i) {
						if(nodelist[z]->getConnection(i) != NULL) {
							std::cerr <<"D:"<<i<<":"<< nodelist[z]->getConnection(i)->getID() << "; ";
						}

					}
					std::cerr << std::endl;
				}
				return 0;
			break;

			case 4:
				if (BACKTRACKING && goal == current_node) {
					BACKTRACKING = false;
					std::cerr << "FOUND AND TURNING" << std::endl;
				// check if robot can move forward, else switch to turn state
				if (current_node->getExplored(dir_robot2frame(r_front,dir)) != 0)
					state = 1; // move forward
				else
					state = 2; // make a turn break;
				}else if(BACKTRACKING){
					std::cerr << "TURNING" << current_node->getsNext()->getTmp() <<std::endl;
					std::cerr << "NODE: "<< current_node->getID() << std::endl;
					dir = dir_frame2robot(current_node->getsNext()->getTmp());
					++turns;
				}
				state = 1;
			break;

		}


	}



	return 0;




}
