#include <iostream>
#include <utility>
#include "Logic.h"
#include <vector>
#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Encoders.h>
#include <movement/Movement.h>
#include "Node.h"
#include <sharps/Distance.h>
#include <cmath>
#include <queue>

static ros::Publisher cmd_pub; //publishes the wanted speed
static ros::Publisher cmd_move; //publishes the wanted move
static ros::Subscriber sharps_sub; //get the sharps information
static ros::Subscriber move_complete_sub; //get "movement complete" messages
static ros::Subscriber encoders; //get delta_encoders

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
const int RIGHT = 1;
const int LEFT = -1;
static double forwardCoeff = .5;
static double turnCoeff = 0;
const double forwardStepSize = .01;
static double angle=0;
static double straightSpeed=1.5;
static double ticks2;
static double ticks1;

//distance threshold for each sharp
static const double sharpsThresholds[6]={
		10., //f
		22.,
		22.,
		17., //r
		22.,
		22.
};

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
/*
int exists(double q,int i){
	int exists=0;
	if (q<10.) {exists=1;}
	if (q<22. && i!=0 && i!=3) {exists=1;}
	return exists;
}
*/

int normalizeAngle(int angle) {
	angle=(angle+180)%360;
	if(angle<0) angle=360+angle;
	return angle-180;
}


//looking at the actual sharps, give the directions to explore
void addNodeDirection(){
	exploreNode_t n;
	n.nodeIdx=name;
	n.direction=0;
	if((type[1]==0)&&(type[2]==0)) {n.direction=normalizeAngle(orientation-90); listExplore.push_back(n);}
	if((type[4]==0)&&(type[5]==0)) {n.direction=normalizeAngle(orientation+90); listExplore.push_back(n);}
	if (type[0]==0) {n.direction=normalizeAngle(orientation); listExplore.push_back(n);}
}


//creates a new node
void NewNode(){
	Node newNode; name++; newNode.setName(name);
	newNode.addParent(actualNode); actualNode=name;
	newNode.setType(type);
	newNode.setOrientation(orientation);
	addNodeDirection();
	nodes.push_back(newNode);
	std::cerr << "New node "<< name <<' '<<" state "<<' ';
	for(int i=0; i<6; ++i) std::cerr<<type[i]<<' ';
	std::cerr << "orientation "<<orientation<< std::endl;
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
//first open loop method
	differential_drive::Speed spd;
	spd.W1 = 2.;
	spd.W2 = 2.;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);

//second PD-looped method
/*	differential_drive::Speed spd;
	double kp = 0.09 ;
	double kd = 0.1;

	angle+=ticks2-ticks1;

spd.W1 = straightSpeed + kp * angle + kd * (ticks2-ticks1);
spd.W2 = straightSpeed - kp * angle - kd * (ticks2-ticks1);
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);

*/
}


//function to follow a wall on right or left (+1 or -1)
void followWall(int wall){

	//this method derives the control of the motors
	differential_drive::Speed spd;//the motors like in fakemotors
	const double IR_rightD = 16.2;		// distance between ch4 and ch8.
	const double K_forward = 0.5; 			// Control coefficient --> ask GB.
	const double Wheel2Wall_D = 5; 		// distance from the wheel to a wall.
	const double k_theta = 8;			// Control coefficient --> Ask GB.
	const double RobotSpeed = 5;		// Constant speed of the robot.
	const double MaxFowardAngle = 0.14;	// Max angle where the robot is allowed to move forward in radians.

	if(wall==RIGHT)
		setState(FOLLOW_R);
	else if(wall==LEFT) 
		setState(FOLLOW_L);
	else {
		std::cerr<<"Weird followWall arg: "<<wall<<std::endl;
		return;
	}

	// Distance of sensor 1 and 2.
	double FrontSensor,RearSensor, forwardSensor;


	// Getting the info from the sensors.
	if (wall==RIGHT){
		FrontSensor = sharpsDistance[1]; // front sensor
		RearSensor = sharpsDistance[2]; // back sensor
	}
	else {
		FrontSensor = sharpsDistance[5]; // front sensor
		RearSensor = sharpsDistance[4]; // back sensor
	}

  forwardSensor = sharpsDistance[0]; //Sensor on front of robot

	double Right_Center_WallD = 0;
	double Robot2Wall_Angle = 0;

	Robot2Wall_Angle = -atan( (FrontSensor-RearSensor)/(IR_rightD) ); // changed to d.
	Right_Center_WallD = (FrontSensor+RearSensor)*0.5*cos(Robot2Wall_Angle); // distance to the wall.

	double turn=turnCoeff;//active theta-control if near wall only
	double forward=accel();//active x-control if

	// While it is turning, decellerate
	if (Robot2Wall_Angle>MaxFowardAngle || -Robot2Wall_Angle>MaxFowardAngle)
	{forward=deccel(true);turn=accelTurn();}
	// If it is not near a wall, remove turning, accelerate
	else if (abs(Right_Center_WallD-Wheel2Wall_D)>5)
	{forward=accel();turn=0;}

	if(forwardSensor < 30)
	{
	  double temp = calcDeccelSpeed(forwardSensor);
	  std::cerr << "Decellerating because there is a wall ahead "<< temp << "\n";
	  if (temp < forward)
	    forward = temp;
	}

	std::cerr << "Forward Coeff " << forward << "\n";
	std::cerr << "ForwardCCCCCC " << forwardCoeff << "\n";
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
	//std::cerr << "Speed R: "<< spd.W1 <<", L:"<<spd.W2 << std::endl;
}

//When we are approaching a wall, calculate the speed so we deccelerate.
//This returns percentage of max_speed to move
double calcDeccelSpeed(double forwardSensor)
{
  double accelerationRange = 4;
  if(forwardSensor > accelerationRange + 10.)
    return 1;
  double temp = (forwardSensor - 10.)/accelerationRange;
  return temp < 0 ? 0 : temp;
}


double accel()
{
  if (forwardCoeff >= 1)
  {
    forwardCoeff = 1;
    return 1;
  }

  forwardCoeff += forwardStepSize;
  return forwardCoeff;
}

double deccel(bool turning)
{
  if(turning && forwardCoeff <= .2)
  {
    forwardCoeff = .2;
    return forwardCoeff;
  }
  else if (forwardCoeff <= 0)
  {
    forwardCoeff = 0;
    return forwardCoeff;
  }
  
  forwardCoeff -= forwardStepSize * 2;
  return forwardCoeff;
}

double accelTurn()
{
  if (forwardCoeff >= 1)
  {
    forwardCoeff = 1;
    return 1;
  }

  forwardCoeff += forwardStepSize;
  return forwardCoeff;
}

double deccelTurn()
{
  turnCoeff = 0;
  return turnCoeff;
}

//ask the robot to advance
/*
void Advance(){
	if (type[4]+type[5]==2) {followWall(-1);}
	else if (type[1]+type[2]==2) {followWall(1);}
	else goAhead();
}
*/
void Advance(){
	if (type[4]+type[5]==2) {
		if (type[1]+type[2]!=2) {followWall(-1);}
		else {
			if (sharpsDistance[4]+sharpsDistance[5]<=sharpsDistance[1]+sharpsDistance[2])
			{followWall(-1);}
			else followWall(1);
		}
	}
	else {if (type[1]+type[2]==2) {followWall(1);}
		  else goAhead();}
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
	int angle=normalizeAngle(goal.direction-orientation); orientation=goal.direction;
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
		int newtype=(sharpsDistance[i]<sharpsThresholds[i])?1:0;
		if(newtype!=type[i]) sameState=false;
		type[i]=newtype;
	}

	if(movementState==TURN) return;

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

void encodersHandler(const differential_drive::Encoders &msg){
ticks2=msg.delta_encoder2;
ticks1=msg.delta_encoder1;
}



//main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Logic");
	ros::NodeHandle nh;
	cmd_pub =  nh.advertise<differential_drive::Speed>("/motion/Speed", 1);
	cmd_move =  nh.advertise<movement::Movement>("/simpleMovement/move",1);
	sharps_sub = nh.subscribe("/sharps/Distance/",1,newState);
	move_complete_sub = nh.subscribe("/simpleMovement/moveCompleted",1,movementCompletedHandler);
	//find name of the topic
	encoders = nh.subscribe("/motion/Encoders",1,encodersHandler);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
