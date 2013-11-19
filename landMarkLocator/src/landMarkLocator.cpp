#include <ros/ros.h>
#include <stdio.h>
#include <inputstream>
#include "utilityCode/NoiseReducer.h"
#include <tgmath.h>

//----------------------------
//CONSTANTS
//----------------------------
static int const UNITITIALIZED = 0;
static int const ITINIALIZED = 1;
//Threshold of considering the difference as a landmark
static double const IR_DIFFERENCE_THRESHOLD=1;
static int const FRONT_RIGHT=1;
static int const FRONT_LEFT=2;
static int const REAR_RIGHT=3;
static int const REAR_LEFT=4;

bool runTests = true;

double const FR_IR_VERTICAL_OFFSET=5; //From front to back
double const FL_IR_VERTICAL_OFFSET=5; 
double const RR_IR_VERTICAL_OFFSET=5; 
double const RL_IR_VERTICAL_OFFSET=5; 
double const FR_IR_HORIZONTAL_OFFSET=5; //From side to side
double const FL_IR_HORIZONTAL_OFFSET=5; 
double const RR_IR_HORIZONTAL_OFFSET=5; 
double const RL_IR_HORIZONTAL_OFFSET=5; 
double const PI = 3.141592653589793238462;

double FR_IR_MAGNITUDE;
double FL_IR_MAGNITUDE;
double RR_IR_MAGNITUDE;
double RL_IR_MAGNITUDE;

double FR_IR_THETA;
double FL_IR_THETA;
double RR_IR_THETA;
double RL_IR_THETA;

static int state = UNITITIALIZED;
static NoiseReducer FR_NR, FL_NR, RR_NR, RL_NR;

//Prototypes
void initializeIRPositionings();
void runtests();

void checkForLandmark(const sharps::Distance &msg)
{
  //put through noiseReducers
  FR_NR.updateList(msg.front_r);
  FL_NR.updateList(msg.front_l);
  RL_NR.updateList(msg.rear_l);
  RR_NR.updateList(msg.rear_r);
  //check new values
  if (checkLandmark(FRONT_RIGHT))
    landMarkFrontRight();
  else if (checkLandmark(REAR_RIGHT))
    landmarkRearRight();
  else if (checkLandmark(FRONT_LEFT))
    landMarkFrontLeft();
  else if (checkLandmark(REAR_LEFT))
    landmarkRearLeft();
}

void checkLandmark(int location)
{
  double diff;
  switch(location)
  {
    case FRONT_RIGHT:
      diff = FR_NR.getAverage() - RR_NR.getAverage();
      break;
    case FRONT_LEFT:
      diff = FL_NR.getAverage() - RL_NR.getAverage();
      break;
    case REAR_RIGHT:
      diff = RL_NR.getAverage() - FL_NR.getAverage();
      break;
    case REAR_LEFT:
      diff = RL_NR.getAverage() - FL_NR.getAverage();
      break;
    default:
      cerr << "Wrong argument in /landmarkLocator.cpp/checkLandmark";
  }

  if (diff > IR_DIFFERENCE_THRESHOLD)
  {
    setLandmark(location);
  }
}

void setLandmark(int location)
{
  //TODO:AlecJ:Get odometry in a clean way
  float32 odoX;
  float32 odoY;
  float32 odoTheta;

  float32 IRX;
  float32 IRY;
  float32 IRTheta;

  float32 landMarkX;
  float32 landMarkY;

  //Calc IRtheta
  if(location == FRONT_RIGHT || location == REAR_RIGHT)
    IRTheta = odoTheta - (PI/2);
  else
    IRTheta = odoTheta + (PI/2);
  
  

  float32 IR_GLOBAL_THETA; //Angle from the center of the odometry to the IRSensor
  float32 Magnitude; //Magnitude of the distance from center of robot to IRSensor
  switch(location)
  {
    case FRONT_RIGHT:
      IR_GLOBAL_THETA = odoTheta + FR_IR_THETA; 
      Magnitue = FR_IR_MAGNITUDE; 
      break;
    case FRONT_LEFT:
      IR_GLOBAL_THETA = odoTheta + FL_IR_THETA; 
      Magnitue = FL_IR_MAGNITUDE; 
      break;
    case REAR_RIGHT:
      IR_GLOBAL_THETA = odoTheta + RR_IR_THETA; 
      Magnitue = RR_IR_MAGNITUDE; 
      break;
    case REAR_LEFT:
      IR_GLOBAL_THETA = odoTheta + RL_IR_THETA; 
      Magnitue = RL_IR_MAGNITUDE; 
      break;
    default:
      cerr << "Not a valid location";
  }
  
  //Calc IRX;
  IRX = Magnitude * cos(IR_GLOBAL_THETA);
  IRY = Magnitude * sin(IR_GLOBAL_THETA);

  float32 LAZER_GLOBAL_THETA = (location == FRONT_RIGHT || location == FRONT_LEFT) ? odoTheta + (PI/2) : odoTheta - (PI/2);

  float32 x = IRX + IRReading * cos(theta);
  float32 y = IRY + IRReading * sin(theta);


}

void initialize()
{
  FR_NR = new NoiseReducer;
  FL_NR = new NoiseReducer;
  RL_NR = new NoiseReducer;
  RR_NR = new NoiseReducer;
  FR_NR.initialize();
  FL_NR.initialize();
  RL_NR.initialize();
  RR_NR.initialize();
  initializeIRPositionings();
  //
  //Change the paramaters here if needed
  //
  state = INITINIALIZED;
}

float32 getHypotenuse(float32 side1, float32 side2);
{
  return sqrt(math::pow(side1, 2) + math::pow(side2, 2));
}

float32 getIRTheta(float32 side1, float32 side2, float32 hypotenus)
{
  //cos(r) = (mag^2 + w^2 - h^2)/ 2 (mag * w)
  float32 temp = math::pow(hypotenus, 2) + math::pow(side2, 2) - math::pow(side1, 2)
  float32 temp2 = 2 * hypotenus * side2
  temp = temp/temp2;
  return acos(temp);
}

void initializeIRPositionings()
{
  //Initialize Calculations (that will remain constant)
  FR_IR_MAGNITUDE = getHypotenuse(FR_IR_VERTICAL_OFFSET, FR_IR_HORIZONTAL_OFFSET);
  FL_IR_MAGNITUDE = getHypotenuse(FL_IR_VERTICAL_OFFSET, FL_IR_HORIZONTAL_OFFSET);
  RR_IR_MAGNITUDE = getHypotenuse(RR_IR_VERTICAL_OFFSET, RR_IR_HORIZONTAL_OFFSET);
  RL_IR_MAGNITUDE = getHypotenuse(RL_IR_VERTICAL_OFFSET, RL_IR_HORIZONTAL_OFFSET);

  FR_IR_THETA = getIRTheta(FR_IR_VERTICAL_OFFSET, FR_IR_HORIZONTAL_OFFSET, FR_IR_MAGNITUDE);
  FL_IR_THETA = getIRTheta(FL_IR_VERTICAL_OFFSET, FL_IR_HORIZONTAL_OFFSET, FL_IR_MAGNITUDE);
  RR_IR_THETA = getIRTheta(RR_IR_VERTICAL_OFFSET, RR_IR_HORIZONTAL_OFFSET, RR_IR_MAGNITUDE);
  RL_IR_THETA = getIRTheta(RL_IR_VERTICAL_OFFSET, RL_IR_HORIZONTAL_OFFSET, RL_IR_MAGNITUDE);
}

int main(int argc, char** argv)
{
  if (runTests)
  {
    runtests();
    return;
  }
    
  if (state == UNITITIALIZED)
    initialize();
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  sharps_sub = nh.subscribe("/sharps/Distance",1,checkForLandmark);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}



void runtests()
{
  initialize();
  cerr << "Magnitude = " << FR_IR_MAGNITUDE;
  cerr << "FR Angle = " << FR_IR_THETA;
}


