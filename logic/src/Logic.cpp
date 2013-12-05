#include <iostream>
#include <utility>
#include "Logic.h"
#include <vector>
#include <ros/ros.h>
#include <differential_drive/Speed.h>
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



static coef=0.05*M_PI;

static ros::Subscriber encoders;

static std::vector<NodeExplore> listNodeExplored;

static std::vector<double,double> listNodeToExplore;

//distance threshold for each sharp
static const double sharpsThresholds[6]={
		12., //f
		23.,
		23.,
		12., //r
		23.,
		23.
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
	updateNodeExplore();
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
	const double K_forward = 0.5; 			// Control coefficient --> ask GB.
	const double Wheel2Wall_D = 5.; 		// distance from the wheel to a wall.
	const double k_theta = 6;//8			// Control coefficient --> Ask GB.
	const double RobotSpeed = 4.5;		// Constant speed of the robot.
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

	// Debugg output
	//std::cerr<<FrontRightSensor<<' '<<RearRightSensor<<' '<<w<<"RIGHT: "<<w1<<" LEFT: "<<w2<<' '<<std::endl;

	// Publish into the speed topic.

	spd.W1 = w1;
	spd.W2 = w2;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);
	//std::cerr << "Speed R: "<< spd.W1 <<", L:"<<spd.W2 << std::endl;
}

//ask the robot to advance
/*
void Advance(){
	if (type[4]+type[5]==2) {followWall(-1);}
	else if (type[1]+type[2]==2) {followWall(1);}
	else goAhead();
}
*/
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
	if(listNodeExplored.empty()) {
		setState(FINISHED);
		return;
	}
	exploreNode_t goal = listExplore.back();
	listExplore.pop_back();
	//we'll make milestone 2 later
	int angle=normalizeAngle(goal.direction-orientation); orientation=goal.direction;
	Rotate(angle);
	Advance();
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
		if(newtype!=type[i]) //sameState=false;
		  type[i]=newtype;
	}
	if (type[0]==1){
	  sameState = false;
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





void updateNodeExplore(){

	node=alreadyVisited();

	if (node!=-1){
	  listNodeExplored[node].disposition[-normalizeAngle(orientation-180)/90][0]=actualNodeExplore;
	  listNodeExplored[node].disposition[-normalizeAngle(orientation-180)/90][1]=-distance*cos(orientation);
	  listNodeExplored[node].disposition[-normalizeAngle(orientation-180)/90][2]=-distance*sin(orientation);
	  listNodeExplored[actualNodeEXplore].disposition[normalizeAngle(orientation-180)/90][0]=node;
	  listNodeExplored[actualNodeEXplore].disposition[normalizeAngle(orientation-180)/90][1]=distance*cos(orientation);
	  listNodeExplored[actualNodeEXplore].disposition[normalizeAngle(orientation-180)/90][2]=distance*sin(orientation);
	  robotPosition=listNodeExplore[node].position;
	}
	else{
	  if(name==-1){
	    //not so useful because next case contains it
	    nodeExplore n;
	    name++;
	    n.name=name;
	    n.position=(0,0);
	    node=name;
	  }
	  else{
	    nodeExplore n;
	    name++;
	    n.name=name;
	    n.position=robotPosition;
	    n.disposition[-normalizeAngle(orientation-180)/90][0]=actualNodeExplore;
	    n.disposition[-normalizeAngle(orientation-180)/90][1]=-distance*cos(orientation);
	    n.disposition[-normalizeAngle(orientation-180)/90][2]=-distance*sin(orientation);
	    listNodeExplored.push_back(n);
	    listNodeExplored[actualNodeEXplore].disposition[normalizeAngle(orientation-180)/90][0]=name;
	    listNodeExplored[actualNodeEXplore].disposition[normalizeAngle(orientation-180)/90][1]=distance*cos(orientation);
	    listNodeExplored[actualNodeEXplore].disposition[normalizeAngle(orientation-180)/90][2]=distance*sin(orientation);
	    node=name;
	  }
	}
	actualNodeExplore=node;
	addNodeDirection();	
	distance=0;
	
}

double alreadyVisited(){

	dist=INFINITY;
	double node =-1;
	
	if (nodeExplore.size()==0){return -1;}
	
	for (int i=0;i<listNodeExplored.size();i++){
	  
	  if(hypot(robotPosition[0]-listNodeExplored[i].position[0],robotPosition[1]-listNodeExplored[i].position[1])<dist){
	    
	    dist=hypot(robotPosition[0]-listNodeExplored[i].position[0],robotPosition[1]-listNodeExplored[i].position[1]);
	    
	    node=i;
	  }
	  
	  
	  if (dist<0.1){return node;}
	  
	  
	  return -1;
	}
}


void goToNeighbourNode(double node){

  orientation=0;

  for (int i=0;i<3;i++){
    if (listNodeExplore[actualNode].disposition[i][0]==node){
      orientation=normalizeAngle(-90*i);
    }
  }
  Rotate(orientation);
  while(acualNode!=node){
    Advance();
  }
}

bool belongsTo(double nb,std::vector<double> vect){
  for(int i=0;i<vect.size();i++){
    if (vect[i]==nb) return true;
  }
  return false;
}

std::vector<double> findPath(double node, double goal, std::vector<double> liste){

  std::vector<double[2]> distances;
  std::vector<std::vector<double>> listOfLists;
  
  for (int i=0;i<3;i++){
    if (listNodeExplored[node].disposition[i][0]!=-1){
      if (listNodeExplored[node].disposition[i][0]==goal){
	distances.push_back(liste[0]+abs(listNodeExplored[node].disposition[i][1]+listNodeExplored[node].disposition[i][2]),i );
	list=liste.pushback(listNodeExplored[node].disposition[i][0]);
	list[0]+=abs(listNodeExplored[node].disposition[i][1]+listNodeExplored[node].disposition[i][2]);
	listOfLists.push_back(list);
      }
      else    {if (belongsTo(listNodeExplored[node].disposition[i][0],liste)){
	  distances.push_back(INFINITY,i);
	  list=liste;list[0]=INFINITY;
	  listOfList.pushback(list);
	}
	else { list=liste.push_back(listNodeExplored[node].disposition[i][0]);
	  list[0]+= abs(listNodeExplored[node].disposition[i][1]+listNodeExplored[node].disposition[i][2]);
	  std::vector<double> fP=findpath(listNodeExplored[node].disposition[i][0],goal,list);
	  distances.push_back(fP[0],i);
	  listOfLists.pushback(fP);
	  
	}
      }
    }
  }


  if (distances.Empty()){list=liste;list[0]=INFINITY;return list;}
  else {N=distances.size();pos=0;min=INFINITY;
    for (int i=0;i<N;i++){if (distances[i][0]<min){pos=i;  min=distances[i][0];}}}
  return listOfList[pos];
}

void goToNode(double goal){ //mettre goal en variable globale!!!

  std::vector<double> liste; liste.push_back(0.01);liste.push_back(actualNode);
  std::vector<double> path=findPath(actualNode,goal,liste);
  
  for (int i=0;i<path.size()-2;i++){
    goToNeighbourNode(path[i+2]);
  }
  
  if(actualNode==goal){dire je suis arriv�}
  
}

void EncodersHandler(const Encoders &msg ){
  // Get the distance the robot is advancing
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
	encoder = nh.subscribe("Encoders",1,EncodersHandler);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
