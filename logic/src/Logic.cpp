#include <iostream>
#include <utility>
#include "Logic.h"
#include <vector>
#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <movement/Movement.h>
#include "Node.h"
#include "nodeExplore.h"
#include <sharps/Distance.h>
#include <std_msgs/Empty.h>
#include <cmath>
#include <differential_drive/Encoders.h>

static ros::Publisher cmd_pub; //publishes the wanted speed
static ros::Publisher cmd_move; //publishes the wanted move
static ros::Subscriber sharps_sub; //get the sharps information
static ros::Subscriber obj_sub; //get the sharps information
static ros::Subscriber move_complete_sub; //get "movement complete" messages
static ros::Subscriber encoders;

static std::vector<exploreNode_t> listExplore; //list of nodes to explore <name,direction>
static int name = -1; //future node name
static int actualNode=0; //actual node position of the robot
static int orientation; //actual robot orientation
static std::vector<Node> nodes; //list of nodes discovered for every shift of sharps

static int type[6];//type of the actual state
static double sharpsDistance [6]; //distance in each sharps

static double distance_=0;//distance the robot went from one node
static double robotPosition[2]={0,0};//position of the robot

using namespace std;

static double coef=0.05*M_PI/360;//to multiply by the ticks
static std::vector<nodeExplore> listNodeExplored;//second list of explored nodes for every wall encountered

bool verbose_(false);

//distance threshold for each sharp
static const double sharpsThresholds[6]={
		12., //front
		23.,
		23.,
		12., //rear
		23.,
		23.
};

//to set the state of the robot
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

//to set the state of the robot
void setState(movementState_t s) {
	if(s!=movementState) {
		std::cerr<<states[movementState]<<" -> "<<states[s]<<std::endl;
		movementState=s;
	}
}


//put the angle between -pi and pi
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
	if((type[1]==0)&&(type[2]==0)) {
		n.direction=normalizeAngle(orientation-90); listExplore.push_back(n);
	}
	if((type[4]==0)&&(type[5]==0)) {
		n.direction=normalizeAngle(orientation+90); listExplore.push_back(n);
	}
	if (type[0]==0) {
		n.direction=normalizeAngle(orientation); listExplore.push_back(n);
	}
	if ( (type[0]==1) && (type[1]==1)&&(type[2]==1)&&(type[4]==1)&&(type[5]==1)){
		n.direction=normalizeAngle(orientation+180); listExplore.push_back(n);
	}
}


//creates a new node
void NewNode(){
	Node newNode;
	updateNodeExplore();
	newNode.setName(name);
	newNode.addParent(actualNode); actualNode=name;
	newNode.setType(type);
	newNode.setOrientation(orientation);
	nodes.push_back(newNode);
	std::cerr << "New node mapping"<< name <<' '<<" state "<<' ';
	for(int i=0; i<6; ++i) std::cerr<<type[i]<<' ';
	std::cerr << "orientation "<<orientation<< std::endl;
}




//ask the robot to rotate
void Rotate(int angle){
	if (angle!=0){
	setState(TURN);
	movement::Movement move;
	move.turn = true;
	move.magnitude = angle*M_PI/180.0;
	move.header.stamp = ros::Time::now();
	cmd_move.publish(move);
	}
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
	const double Wheel2Wall_D = 4.; 		// distance from the wheel to a wall.
	const double k_theta = 6;//8			// Control coefficient --> Ask GB.
	const double RobotSpeed = 2;		// Constant speed of the robot.
	const double MaxFowardAngle = 0.15;//0.14	// Max angle where the robot is allowed to move forward in radians.

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
	double w = 0.2*wall*(K_forward*(Right_Center_WallD-Wheel2Wall_D)*forward - k_theta*Robot2Wall_Angle*turn);
	if(verbose_) std::cerr<<"w: "<<w<<std::endl;

	double v = forward*RobotSpeed;

	//speeds
	double w1 = 1*(-w+v);//right wheel
	double w2 = 1*(v-w); //left wheel

	// Debugg output


	// Publish into the speed topic.

	spd.W1 = w1;
	spd.W2 = w2;
	spd.header.stamp = ros::Time::now();
	cmd_pub.publish(spd);
	//std::cerr << "Speed R: "<< spd.W1 <<", L:"<<spd.W2 << std::endl;
}

//to set the way the robot advances
void Advance(){

	if (type[4]+type[5]==2 && sharpsDistance[4]<12) {
		if (type[1]+type[2]!=2) {followWall(-1);}//if wall on left follow left
		else {
			if (sharpsDistance[4]+sharpsDistance[5]<=sharpsDistance[1]+sharpsDistance[2])
			{followWall(-1);}
			else followWall(1);
			//if two walls, follow the nearest one
		}
	}
	else {if (type[1]+type[2]==2 && sharpsDistance[1]<12) {followWall(1);}
	//follow right
		  else goAhead();//if nothing go straight without wall
	}
}

//ask the robot to move
void Move(){
	if(listNodeExplored.empty()) {
		setState(FINISHED);
		return;
	}
	exploreNode_t goal = listExplore.back();
	listExplore.pop_back();

	int angle=normalizeAngle(goal.direction-orientation); orientation=goal.direction;

	cerr<<"angle: "<<angle;
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
		if(newtype!=type[i]) {type[i]=newtype;}
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

	int node=alreadyVisited();
	cerr<<"node"<<node;

	if (node!=-1){
	  listNodeExplored[node].disposition[-normalizeAngle(orientation-180)/90][0]=actualNode;
	  listNodeExplored[node].disposition[-normalizeAngle(orientation-180)/90][1]=-1*distance_*cos(orientation);
	  listNodeExplored[node].disposition[-normalizeAngle(orientation-180)/90][2]=-1*distance_*sin(orientation);
	  listNodeExplored[actualNode].disposition[normalizeAngle(orientation-180)/90][0]=node;
	  listNodeExplored[actualNode].disposition[normalizeAngle(orientation-180)/90][1]=distance_*cos(orientation);
	  listNodeExplored[actualNode].disposition[normalizeAngle(orientation-180)/90][2]=distance_*sin(orientation);
	  robotPosition[0]=listNodeExplored[node].position[0];
	  robotPosition[1]=listNodeExplored[node].position[1];
	}
	else{
	  nodeExplore n;

	  if(name==-1){
		  n.init();
	    n.name=0;
	    	for(int i=0;i<4;i++){
	    		n.disposition[i][0]=-1;
	    	}
	    name++;
	    n.name=name;
	    n.position[0]=0;
	    n.position[1]=0;
	    node=name;

	    listNodeExplored.push_back(n);

	  }
	  else{

		n.init();
	    n.name=0;
	    for(int i=0;i<4;i++){
	    	n.disposition[i][0]=-1;
	    }
	    name++;
	    n.name=name;
	    n.position[0]=robotPosition[0];n.position[1]=robotPosition[1];
	    n.disposition[-normalizeAngle(orientation-180)/90][0]=actualNode;
	    n.disposition[-normalizeAngle(orientation-180)/90][1]=-distance_*cos(orientation);
	    n.disposition[-normalizeAngle(orientation-180)/90][2]=-distance_*sin(orientation);
	    listNodeExplored.push_back(n);
	    listNodeExplored[actualNode].disposition[normalizeAngle(orientation-180)/90][0]=name;
	    listNodeExplored[actualNode].disposition[normalizeAngle(orientation-180)/90][1]=distance_*cos(orientation);
	    listNodeExplored[actualNode].disposition[normalizeAngle(orientation-180)/90][2]=distance_*sin(orientation);
	    node=name;
	  }
	}
	actualNode=node;
	addNodeDirection();
	cout<<"Distance is"<<distance_<<endl;
	distance_=0;
	
}

int alreadyVisited(){

	double dist=INFINITY;
	int node =-1;
	int test = listNodeExplored.size();
	cout<<"test "<< test<<endl;
	if (test==0){
		return -1;
	}
	
	for (int i=0;i<listNodeExplored.size();i++){
	  
	  if( hypot(robotPosition[0]-listNodeExplored[i].position[0],robotPosition[1]-listNodeExplored[i].position[1]) <dist){
	    
	    dist=hypot(robotPosition[0]-listNodeExplored[i].position[0],robotPosition[1]-listNodeExplored[i].position[1]);
	    
	    node=i;
	  }
	  if (dist<0.1){return node;}
	}
	return -1;
}


void goToNeighbourNode(int node){

  orientation=0;

  for (int i=0;i<3;i++){
    if (listNodeExplored[actualNode].disposition[i][0]==node){
      orientation=normalizeAngle(-90*i);
    }
  }
  Rotate(orientation);
  while(actualNode!=node){
    Advance();
  }
}

bool belongsTo(double nb,std::vector<double> vect){
  for(int i=0;i<vect.size();i++){
    if (vect[i]==nb) return true;
  }
  return false;
}


vector<double> findPath(int node, int goal, vector<double> liste){

  vector<double> distances;
  vector< vector<double> > listOfLists;

  for (int i=0;i<3;i++){
    if (listNodeExplored[node].disposition[i][0]!=-1){
      if (listNodeExplored[node].disposition[i][0]==goal){
    	  distances.push_back(liste[0]+abs(listNodeExplored[node].disposition[i][1]+listNodeExplored[node].disposition[i][2]));
    	  vector<double> list=liste;
    	  list.push_back(listNodeExplored[node].disposition[i][0]);
    	  list[0]+=abs(listNodeExplored[node].disposition[i][1]+listNodeExplored[node].disposition[i][2]);
    	  listOfLists.push_back(list);
      }
      else    {
    	  if (belongsTo(listNodeExplored[node].disposition[i][0],liste)){
    		  distances.push_back(INFINITY);
    		  vector<double> list=liste;list[0]=INFINITY;
    		  listOfLists.push_back(list);
    	  }
    	  else {
    		  vector<double> list=liste;
    		  list.push_back(listNodeExplored[node].disposition[i][0]);
    		  list[0]= list[0]+ abs(listNodeExplored[node].disposition[i][1]+listNodeExplored[node].disposition[i][2]);
    		  int node_temp = listNodeExplored[node].disposition[i][0];
    		  std::vector<double> fP=findPath(node_temp,goal,list);
    		  distances.push_back(fP[0]);
    		  listOfLists.push_back(fP);
    	  }
      }
    }
  }

  int pos =0;
  if (distances.empty()){
	  vector<double> list=liste;
	  list[0]=INFINITY;
	  return list;
  }
  else {
	  int N=distances.size();
	  double min=INFINITY;
	  for (int i=0;i<N;i++){
		  if (distances[i]<min){
			  pos=i;
			  min=distances[i];
		  }
	  }
  }
  return listOfLists[pos];
}

void goToNode(double goal){ //mettre goal en variable globale!!!

  std::vector<double> liste; liste.push_back(0.01);liste.push_back(actualNode);
  std::vector<double> path=findPath(actualNode,goal,liste);
  
  for (int i=0;i<path.size()-2;i++){
    goToNeighbourNode(path[i+2]);
  }
  
  if(actualNode==goal){
	  std::cout<<"Robot has arrived to target"<<std::endl;
  }
  
}

void EncodersHandler(const differential_drive::Encoders &msg ){
  // Get the distance the robot is advancing
  int ticks1=msg.delta_encoder1;
  int ticks2=msg.delta_encoder2;
  distance_=distance_ +coef*(ticks1+ticks2);
  if(orientation==0){robotPosition[0]+=distance_;}
  if(orientation==-90){robotPosition[1]-=distance_;}
  if(orientation==-180){robotPosition[0]-=distance_;}
  if(orientation==90){robotPosition[1]+=distance_;}
}




//main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Logic");
	ros::NodeHandle nh;

	sharps_sub = nh.subscribe("sharps/Distance/",10,newState);
	obj_sub = nh.subscribe("objdetect/spotted",10,objdetectHandler);
	move_complete_sub = nh.subscribe("simpleMovement/moveCompleted",10,movementCompletedHandler);
	encoders = nh.subscribe("motion/Encoders",10,EncodersHandler);
	cmd_pub =  nh.advertise<differential_drive::Speed>("motion/Speed", 10);
	cmd_move =  nh.advertise<movement::Movement>("simpleMovement/move",10);


	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
