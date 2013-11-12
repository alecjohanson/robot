#include <iostream>
#include <utility>
#include "Logic.h"
#include <vector>
#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <movement/Movement.h>
#include "Node.h"
#include <sharps/Distance.h>
#include <cmath>

static ros::Publisher cmd_pub; //publishes the wanted speed
static ros::Publisher cmd_move; //publishes the wanted move
static ros::Subscriber sharps_sub; //get the sharps information
static ros::Subscriber move_complete_sub; //get "movement complete" messages

static std::vector<exploreNode_t> listExplore; //list of nodes to explore <name,direction>
static int name = -1; //future node name
static int actualNode=0; //actual node position of the robot
static int orientation; //actual robot orientation
static std::vector<Node> nodes; //list of nodes discovered
//static bool newstate; //is the robot in a new state
static int type[6];//type of the actual state
static double sharpsDistance [6]; //distance in each sharps
//static bool turning=false;//to tell if the robot is turning or not
//static int initialize=1;//to tell if we are initializing

typedef enum{
	INIT=0,
	FOLLOW_L,
	FOLLOW_R,
	TURN,
	STRAIGHT,
	FINISHED
} movementState_t;
static const char *states[]={
		"INIT",
		"FOLLOW_L",
		"FOLLOW_R",
		"TURN",
		"STRAIGHT",
		"FINISHED"
};
movementState_t movementState;

void setState(movementState_t s) {
	if(s!=movementState) {
		std::cerr<<states[movementState]<<" -> "<<states[s]<<std::endl;
		movementState=s;
	}
}

// returns if there is a wall or not
int exists(double q){
	int exists=0; if (q<10) exists=1;
	return exists;}


//looking at the actual sharps, give the directions to explore
void addNodeDirection(){
	exploreNode_t n;
	n.nodeIdx=name;
	n.direction=0;
	if((type[1]==0)&&(type[2]==0)) {n.direction=-90; listExplore.push_back(n);}
	if((type[4]==0)&&(type[5]==0)) {n.direction=90; listExplore.push_back(n);}
	if (type[0]==0) {n.direction=0; listExplore.push_back(n);}
}


//creates a new node
void NewNode(){
	Node newNode; name++; newNode.setName(name);
	newNode.addParent(actualNode); actualNode=name;
	newNode.setType(type);
	newNode.setOrientation(orientation);
	addNodeDirection();
	nodes.push_back(newNode);
	std::cerr << name <<' ';
	for(int i=0; i<6; ++i) std::cerr<<type[i]<<' ';
	std::cerr<<orientation<< std::endl;
}




//ask the robot to rotate
void Rotate(int angle){
	setState(TURN);
	movement::Movement move;
	move.turn = true;
	move.magnitude = angle*M_PI/180.0;
	move.header.stamp = ros::Time::now();
	cmd_move.publish(move);
}


//ask the robot to go ahead
void goAhead(){
	setState(STRAIGHT);
	differential_drive::Speed spd;
	spd.W1 = 2.;
	spd.W2 = 2.;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);

}


//function to follow a wall on right or left (+1 or -1)
void followWall(int wall){

	//this method derives the control of the motors
	differential_drive::Speed spd;//the motors like in fakemotors
	const double IR_rightD = 16.2;		// distance between ch4 and ch8.
	const double K_forward = 0.5; 			// Control coefficient --> ask GB.
	const double Wheel2Wall_D = 5; 		// distance from the wheel to a wall.
	const double k_theta = 15;			// Control coefficient --> Ask GB.
	const double RobotSpeed = 5;		// Constant speed of the robot.
	const double MaxFowardAngle = 0.08;	// Max angle where the robot is allowed to move forward in radians.

	if(wall==1)
		setState(FOLLOW_R);
	else if(wall==-1)
		setState(FOLLOW_L);
	else {
		std::cerr<<"Weird followWall arg: "<<wall<<std::endl;
		return;
	}

	// Distance of sensor 1 and 2.
	double FrontSensor,RearSensor;


	// Getting the info from the sensors.
	if (wall==1){
		FrontSensor = sharpsDistance[1]; // front sensor
		RearSensor = sharpsDistance[2]; // back sensor
	}
	else {
		FrontSensor = sharpsDistance[5]; // front sensor
		RearSensor = sharpsDistance[4]; // back sensor
	}

	double Right_Center_WallD = 0;
	double Robot2Wall_Angle = 0;

	Robot2Wall_Angle = -atan( (FrontSensor-RearSensor)/(IR_rightD) ); // changed to d.
	Right_Center_WallD = (FrontSensor+RearSensor)*0.5*cos(Robot2Wall_Angle); // distance to the wall.

	double turn=1;//active theta-control if near wall only
	double forward=1;//active x-control if

	// While it is turning, dont go foward.
	if (Robot2Wall_Angle>MaxFowardAngle || -Robot2Wall_Angle>MaxFowardAngle)
	{forward=0.2;turn=1;}
	// If it is not near a wall, remove turning
	else if (abs(Right_Center_WallD-Wheel2Wall_D)>5)
	{forward=1;turn=0;}

	// Control
	double w = wall*(K_forward*(Right_Center_WallD-Wheel2Wall_D)*forward - k_theta*Robot2Wall_Angle*turn);

	double v = forward*RobotSpeed;
	//speeds
	double w1 = 1*(-w+v);//right wheel
	double w2 = 1*(v+w); //left wheel

	// Debugg output
	//std::cerr<<FrontRightSensor<<' '<<RearRightSensor<<' '<<w<<"RIGHT: "<<w1<<" LEFT: "<<w2<<' '<<std::endl;

	// Publish into the speed topic.

	spd.W1 = w1;
	spd.W2 = w2;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);
	std::cerr << "Speed R: "<< spd.W1 <<", L:"<<spd.W2 << std::endl;
}

//ask the robot to advance
void Advance(){
	if (type[4]+type[5]==2) {followWall(-1);}
	else if (type[1]+type[2]==2) {followWall(1);}
	else goAhead();
}


//ask the robot to move
void Move(){
	if(listExplore.empty()) {
		setState(FINISHED);
		return;
	}
	exploreNode_t goal = listExplore.back();
	listExplore.pop_back();
	//we'll make milestone 2 later
	int angle=goal.direction-orientation; orientation=goal.direction;
	if(angle) Rotate(angle);
	else Advance();
}


//update the state and detect if it is new
void newState (const sharps::Distance &msg){
	sharpsDistance[0]=msg.front;
	sharpsDistance[1]=msg.front_r;
	sharpsDistance[2]=msg.rear_r;
	sharpsDistance[3]=msg.rear;
	sharpsDistance[4]=msg.rear_l;
	sharpsDistance[5]=msg.front_l;

	bool sameState=true;
	for(int i=0; i<6; ++i) {
		int newtype=exists(sharpsDistance[i]);
		if(newtype!=type[i]) sameState=false;
		type[i]=newtype;
	}

	if(movementState!=INIT && sameState) {
		switch(movementState) {
		case FOLLOW_L:
			followWall(-1);
			break;
		case FOLLOW_R:
			followWall(1);
			break;
		case TURN:
			break;
		case STRAIGHT:
			goAhead();
			break;
		default: break;
		}
	} else {
		NewNode();
		Move();
	}
}

void movementCompletedHandler(const movement::Movement &msg) {
	if(msg.turn && movementState==TURN)
		Advance();
}


//main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Logic");
	ros::NodeHandle nh;
	ros::Publisher cmd_pub =  nh.advertise<differential_drive::Speed>("/motion/Speed", 1);
	ros::Publisher cmd_move =  nh.advertise<movement::Movement>("/simpleMovement/move",1);
	sharps_sub = nh.subscribe("/sharps/Distance/",1,newState);
	move_complete_sub = nh.subscribe("/simpleMovement/moveCompleted",1,movementCompletedHandler);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
