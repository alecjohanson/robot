#include "Mapping.h"
#include <iostream>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Encoders.h>
#include <movement/Movement.h>
#include "Node.h"
#include <sharps/Distance.h>
#include <std_msgs/Empty.h>
#include <cmath>

static ros::Publisher cmd_pub; //publishes the wanted speed
static ros::Publisher cmd_move; //publishes the wanted move
static ros::Subscriber sharps_sub; //get the sharps information
static ros::Subscriber obj_sub; //get the sharps information
static ros::Subscriber move_complete_sub; //get "movement complete" messages
static ros::Subscriber odometry;


static int name = -1; //future node name
static int actualNode=0; //actual node position of the robot
static int orientation; //actual robot orientation
static std::vector<Node> nodes; //list of nodes discovered
static std::vector<Node> nodesExplored; //simplified list of nodes
static int type[6];//type of the actual state
static double sharpsDistance [6]; //distance in each sharps
static double ticks1=0;
static double ticks2=0;
static double distance=0;
static double r=0.05;
static double T=360;
static double coef= M_PI*r/T;
static int lastNode=0;

//inferior mapping
static std::vector<exploreNode_t> listExplore; //list of nodes to explore <name,direction>
static std::vector<nodeExplore> nodeExploreInferior;
static std::vector<double[2]> listOfPoints;
static double robotPosition=(0,0);
static int nameExplore=-1;
static int actualNodeExplore=0;

//distance threshold for each sharp
static const double sharpsThresholds[6]={
		12., //f
		15.,
		15.,
		17., //r
		15.,
		15.
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

//method to normalize the angle
int normalizeAngle(int angle) {
	angle=(angle+180)%360;
	if(angle<0) angle=360+angle;
	return angle-180;
}

//treats the nodes list to simplify it and keep the rotation nodes
void nodeListTreatment(){

int N=nodes.size();

for (int i=lastNode+1;i<N;i++){
	//if the direction is the same
	if (nodes[i].getOrientation()==nodes[lastNode].getOrientation()){
		//if the overcome distance is too little (used to get rid of noisy nodes)
		if (nodes[i].getDistance(0)<0.02){
			//node stays the same
			nodesExplored[lastNode].setDistance(nodesExplored[lastNode].getDistance(0)+nodes[i].getDistance(0));
			break;
		}
		//a modifier plus tard pour mettre un noeud sur le mur disparu de gauche
		else{
		nodesExplored[lastNode].setDistance(nodesExplored[lastNode].getDistance(0)+nodes[i].getDistance(0));
		}
	}
	else{//if robot has rotated, put a new node
		lastNode=i;
		nodesExplored.push_back(nodes[i]);
	}
	}
lastNode=N;
}

/*
//a modifier, pour faire les cas ou il y a un petit passage a gauche

bool sameWall(Node n1,Node n2){
int type1[6]=n1.getNodeType();
int type2[6]=n2.getNodeType();
//if there is a followed wall,
if ((type1[4]==type2[4]&&type1[5]==type2[5])||(type1[4]==type2[4]&&type1[5]==type2[5])&&type2[0]!=0){
	return true;
}

return false;
}
*/

int nearestNode(){
double dist=INFINITY;
for (int i=0;i<listOfPoints.size();i++){
	if(std::hypot(robotPosition[0]-listOfPoints[i][0],robotPosition[1]-listOfPoints[i][1])< ){

	}


}
return 0;
}


//create nodeExploreList
void updateListExplore(){
if (nameExplore==-1){

	nodeExplore n;
	n.nodeExplore();
	nameExplore++;
	n.name=nameExplore;

	double t[2]=(0,0);
	listOfPoints.push_back((nameExplore,t));
	nodeExploreInferior.push_back(n);
	addNodeDirection();
	actualNodeExplore=nameExplore;
	distance=0;

}
else{

nodeExplore n;
nameExplore++;
n.name=nameExplore;

robotPosition[0]+=distance*cos(orientation*M_PI/180);
robotPosition[1]+=distance*cos(orientation*M_PI/180);

n.disposition[normalizeAngle((orientation-180)/90)][1]=-distance*cos(orientation*M_PI/180);
n.disposition[normalizeAngle((orientation-180)/90)][2]=-distance*sin(orientation*M_PI/180);

double t[2]=(robotPosition[1],robotPosition[2]);
listOfPoints.push_back((nameExplore,t));
nodeExploreInferior[actualNodeExplore].disposition[orientation/90][0]=nameExplore;
nodeExploreInferior[actualNodeExplore].disposition[orientation/90][1]=distance*cos(orientation*M_PI/180);
nodeExploreInferior[actualNodeExplore].disposition[orientation/90][2]=distance*cos(orientation*M_PI/180);

addNodeDirection();

actualNodeExplore=nameExplore;
distance=0;
}
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
	//je le mets pour un autre
	//addNodeDirection();
	updateListExplore();
	nodes.push_back(newNode);
	if (name==0){nodesExplored.push_back(nodes[0]);}
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
	differential_drive::Speed spd;
	spd.W1 = 2.;
	spd.W2 = 2.;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);

}


//function to follow a wall on right or left (+1 or -1)
void followWall(int wall){//here i played with wheel2wall and ktheta

	//this method derives the control of the motors
	differential_drive::Speed spd;//the motors like in fakemotors
	const double IR_rightD = 16.2;		// distance between ch4 and ch8.
	const double K_forward = 0.4;//0.5 			// Control coefficient --> ask GB.
	const double Wheel2Wall_D = 5.; 		// distance from the wheel to a wall.
	const double k_theta = 8;//8			// Control coefficient --> Ask GB.
	const double RobotSpeed = 4.5;//5		// Constant speed of the robot.
	const double MaxFowardAngle = 0.12;//0.14	// Max angle where the robot is allowed to move forward in radians.

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


	// Publish into the speed topic.

	spd.W1 = w1;
	spd.W2 = w2;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);

}


void Advance(){//here i put a new condition to advance
	if (type[4]+type[5]==2 && sharpsDistance[4]<12) {
		if (type[1]+type[2]!=2) {followWall(-1);}
		else {
			if (sharpsDistance[4]+sharpsDistance[5]<=sharpsDistance[1]+sharpsDistance[2])
			{followWall(-1);}
			else followWall(1);
		}
	}
	else {if (type[1]+type[2]==2 && sharpsDistance[1]<12) {followWall(1);}
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

	if(movementState==FINISHED) return;
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
		//here we set the distance overcome in the node
		nodes[actualNode].setDistance(distance);
		std::cerr << "distance"<<distance<< std::endl;
		distance=0;
		//and we create the new node
		NewNode();
		Move();
	}
}

void objdetectHandler(const std_msgs::Empty &msg) {
	movementState=FINISHED;
	differential_drive::Speed spd;
	spd.W1 = 0.;
	spd.W2 = 0.;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);
}

void movementCompletedHandler(const movement::Movement &msg) {
	if(msg.turn && movementState==TURN)
		Advance();
}

void encodersHandler (const::differential_drive::Encoders &msg){
ticks1=msg.delta_encoder1;
ticks2=msg.delta_encoder2;
distance+=coef*(ticks1+ticks2);
}

//main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Logic");
	ros::NodeHandle nh;
	cmd_pub =  nh.advertise<differential_drive::Speed>("motion/Speed", 1);
	cmd_move =  nh.advertise<movement::Movement>("simpleMovement/move",1);
	sharps_sub = nh.subscribe("sharps/Distance/",1,newState);
	obj_sub = nh.subscribe("objdetect/spotted",1,objdetectHandler);
	move_complete_sub = nh.subscribe("simpleMovement/moveCompleted",1,movementCompletedHandler);
	odometry = nh.subscribe("motion/Encoders",1,encodersHandler);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
