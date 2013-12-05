
//============================================================================
// Name        : navigation.cpp
// Author      : alfred
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include "navigation.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include "../../sharps/src/SensorPublisher.h"
#include <differential_drive/Encoders.h>
#include <std_msgs/Int32.h>
#include <string.h>
#include <sharps/Distance.h>


using namespace differential_drive;

static ros::Publisher cmd_pub;
static ros::Subscriber sharps_sub;
static ros::Subscriber	enc_sub;

// Some constants that are needed, measurements still need to be measured.
// position of the sensors...
const std::pair<double,double> IR1_POS(0.06,-0.1);
const std::pair<double,double> IR2_POS(0.07,0.0);
const std::pair<double,double> IR3_POS(0.06,0.1);
const std::pair<double,double> IR4_POS(-0.06,0.1);
const std::pair<double,double> IR5_POS(-0.07,0.0);
const std::pair<double,double> IR6_POS(-0.06,-0.10);
const double R_Base = 0.21; 							// robot base
const double R_Wheel_Radius = 0.05;
const double R_Max_Speed = 0.1; // maximum speed in meters/s (does not make sense, motor controller is weird)
const double delta = 0.1; // distance to keep from objects

// Global
std::vector<double> IR_last (6,0);
std::vector<double> Speed_last(2,0);
double T;
std::vector<double> Speed_cur(2,0);
double theta_last = 0,I_last = 0;; 
// Get length of vector
double GetVectorLength(std::vector<double>& vector) {
	if (vector[0]==0 && vector[1] == 0) return 0;	
	double length = pow(pow(vector[0],2) + pow(vector[1],2),0.5);
	return length;
}


/* Creates a vector to the object to be avoided, can be a virtual  object.
 * Calculates this by just taking the closest object right now.
 * UPGRADE: should take all information about  objects into account,
 * weighing each one depending on distance and also which direction the robot is
 * heading.
 *
 * Takes input measurements in meters
 * Vectors are defined with x in the robots direction, y = x*Rotate(pi/2)
 * and origo in the robots centre of rotation
 */
std::vector<double> GetObjVector(std::vector<double> IR) {


	// read measurement data

	std::vector<double> ObjVector(2,0);
	std::vector< std::vector<double> > IR_vectors(6);
	double K = 0,C=200; // adjust C to change the response curve
	double alpha = 10;
	double e = 0,eps = 0.0001;
	// calculate IR-object-vectors
	IR_vectors[0].push_back(IR1_POS.first); IR_vectors[0].push_back(IR1_POS.second-IR[0]);
	IR_vectors[1].push_back(IR2_POS.first+IR[1]); IR_vectors[1].push_back(IR2_POS.second);
	IR_vectors[2].push_back(IR3_POS.first); IR_vectors[2].push_back(IR3_POS.second+IR[2]);
	IR_vectors[3].push_back(IR4_POS.first); IR_vectors[3].push_back(IR4_POS.second+IR[3]);
	IR_vectors[4].push_back(IR5_POS.first-IR[4]); IR_vectors[4].push_back(IR5_POS.second);
	IR_vectors[5].push_back(IR6_POS.first); IR_vectors[5].push_back(IR6_POS.second-IR[5]);

	// weigh the vectors from the IR measurements
	double sum = 0;
	for (int i = 0; i<6; ++i) {
	// weighing, todo: make it care less about abjects behind you

		if (GetVectorLength(IR_vectors[i]) < 0.25 && GetVectorLength(IR_vectors[i]) > 0.0 && IR[i] != 0) { 	
			std::cerr << i<<":: LENGTH: "<< GetVectorLength(IR_vectors[i]) << std::endl;
			// check to see that the value does not fluctates too much
			// this implies the sensor does not measure anything
			if (std::abs(IR_last[i]-GetVectorLength(IR_vectors[i])) > 0.010) {
	
			}else{			
	
				// dont care about measurements that is higher 
				// calculate gain, which depends on the angle as to not care that much about 
				// objects that are behind the robot.		
				double vecangle = (M_PI - std::abs(atan2(IR_vectors[i][1],IR_vectors[i][0])))/M_PI;
				if (vecangle == 0) break; // dont care about stuff behind the robot				
				vecangle = (1-pow(M_E,-alpha*pow(vecangle,2))/vecangle);					
				//std::cerr << i <<":: Length" << GetVectorLength(IR_vectors[i]) << ": Angle: " \
					<< vecangle << std::endl;
					
				ObjVector[0] += vecangle*IR_vectors[i][0];
				ObjVector[1] += vecangle*IR_vectors[i][1];
				++sum;
			}
			// update
			IR_last[i] = GetVectorLength(IR_vectors[i]);
			ObjVector[0] /= sum;
			ObjVector[1] /= sum;
		}
	}
	

	// weigh according to distance to object
	// normalize
	e = std::abs(delta-GetVectorLength(ObjVector));
	double scale = GetVectorLength(ObjVector);	
	
	if (scale==0) return ObjVector;
 	
	ObjVector[0] /= scale;
	ObjVector[1] /= scale;	
	K = (pow(M_E,-C*pow(e,2)));
	ObjVector[0] = ObjVector[0]*K;
	ObjVector[1] = ObjVector[1]*K;

	return ObjVector;

}


/* Gets a vector to goal and return appropriate wheel speeds in angular velocities (Right wheel, Left Wheel)
 * length of goal vector is robot speed.
 */
std::vector <double> FollowVector (std::vector<double> U) {
	std::vector<double> Speed_vector (2,0);
	double eps = 0.06; // distance to virtual point which we assume \
				to have direct control over, smaller better, \
				to small->not good, turns to fast.
	double theta_d = atan2(U[1],U[0]); // Desired theta, used atan2 as to \
						always return value betwix pi and -pi, otherwise fucks up
	double v = cos(theta_d)*U[0] + sin(theta_d)*U[1]; 	   // forward velocity
	double w = 1/eps*(-sin(theta_d)*U[0] + cos(theta_d)*U[1]); // angular velocity
	double Vr = v + w*R_Base; // right wheel speed
	double Vl = v - w*R_Base; // left wheel speed
	double wr = Vr/R_Wheel_Radius; // right angular velocity
	double wl = Vl/R_Wheel_Radius; // left angular velocity
	//std::cerr << -sin(theta_d)*U[0] <<":" << cos(theta_d)*U[1] << std::endl;

	//v = 0;
	// make a PID controller!!!
	double KI = 0.0, KD = 0.00, KP = 2; 
	double e_dot = (theta_d-theta_last)/T;
	double e_s = T*theta_d + I_last;
	w = KP*theta_d + KI*e_s + KD*e_dot;
	v = w;
	double p = 0.20;
	w = w *(1-p);
	v = v *p;
	if (GetVectorLength(U) == 0) w = 0;
	I_last = e_s;
	theta_last = theta_d;
	// do some saturation
	if(I_last > 1) I_last = 1;
	//v = 0.05;
	Vr = w*R_Base/2 + v; // right wheel speed
	Vl = -w*R_Base/2 + v; // left wheel speed
	wr = Vr/R_Wheel_Radius; // right angular velocity
	wl = Vl/R_Wheel_Radius; // left angular velocity

	std::cerr << e_s <<":" << e_dot << "__" << theta_d<<std::endl;
	Speed_vector[0] = -wr/7;
	Speed_vector[1] = -wl/7;

	return Speed_vector;
}

//Callback function for the "/encoder" topic. Prints the encoder value and randomly changes the motor speed.
void receive_encoder(const Encoders::ConstPtr &msg)
{
  static ros::Time t_start = ros::Time::now();
  double left = msg->delta_encoder2;
  double right = msg->delta_encoder1;
  int timestamp = msg->timestamp;
  //std::cerr << "Right: "<<right*R_Wheel_Radius*2.52<<", Left: "<< left*R_Wheel_Radius*2.52 << std::endl;
  Speed_cur[0] = left*R_Wheel_Radius*2.52;
  Speed_cur[1] = right*R_Wheel_Radius*2.52;

}




void test(const sharps::Distance &msg){

  	Speed spd;//the motors like in fakemotors
  
  	// Getting the info from the sensors (absolute).
  	std::vector<double> IR_measurements (6,0);
	if (msg.front_r != 0 && msg.front_r != MaxDist) IR_measurements[0] = (msg.front_r+Sharp2Wheel_FrontRightD)*0.01; 
  	if (msg.front != 0 && msg.front != MaxDist) IR_measurements[1] = (msg.front+ Sharp2Edge_FrontD)*0.01; 
  	if (msg.front_l != 0 && msg.front_l != MaxDist) IR_measurements[2] = (msg.front_l+Sharp2Wheel_FrontLeftD)*0.01; 
  	if (msg.rear_l != 0 && msg.rear_l != MaxDist) IR_measurements[3] = (msg.rear_l+Sharp2Wheel_RearLeftD)*0.01; 
  	if (msg.rear != 0 && msg.rear != MaxDist) IR_measurements[4] = (msg.rear+ Sharp2Edge_RearD)*0.01; 
  	if (msg.rear_r != 0 && msg.rear_r != MaxDist) IR_measurements[5] = (msg.rear_r+Sharp2Wheel_RearRightD)*0.01; 
	
	std::vector <double> ObjDist (2,0);
	ObjDist = GetObjVector(IR_measurements); // Get obj vector from measurements
	// multiply and take the negative to get velocity vector
	ObjDist[0] = -ObjDist[0]*R_Max_Speed;
	ObjDist[1] = -ObjDist[1]*R_Max_Speed;

	// try follow wall
	double temp1 = ObjDist[0];
	double temp2 = ObjDist[1];
	
	std::vector<double> FWC (2,0);
	FWC[1] = -ObjDist[0];
	FWC[0] = ObjDist[1];
	std::vector<double> FWCC (2,0);
	FWCC[1] = ObjDist[0];
	FWCC[0] = ObjDist[1];
	std::vector<double> GOAL (2,0);
	GOAL[0] = 0.005;
	GOAL[1] = -0.005;
		
	ObjDist = FWC;
	//ObjDist[0] = 0.00;
	//ObjDist[1] = -0.1;


	if(!std::isnan(GetVectorLength(ObjDist))) {
		std::vector<double> Speed = FollowVector (ObjDist); 


		// make the speed transitions smooth
		// average feels good to do
		/*if (GetVectorLength(Speed) != 0) {
			Speed[0] = (Speed[0]+Speed_cur[0])/2;
			Speed[1] = (Speed[1]+Speed_cur[1])/2;
		}
		 */

	
		// Publish into the speed topic.
		std::cerr << "LENGTH: " << GetVectorLength(ObjDist) << ", ANGLE:" << \
				180.0/M_PI*atan2(ObjDist[1],ObjDist[0]) << \
				"Speed R: "<< Speed[1] <<", L:"<<Speed[0] << std::endl; 


		  spd.W1 = Speed[1]; //right wheel
		  spd.W2 = Speed[0];
		  spd.header.stamp = ros::Time::now();
		 // cmd_pub.publish(spd);
	}


}




int main(int argc, char** argv)
{

	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	cmd_pub = nh.advertise<Speed>("/motion/Speed", 1);
	sharps_sub = nh.subscribe("/sharps/Distance/",1,test);
	enc_sub = nh.subscribe("/motion/Encoders", 1000, receive_encoder);
	


   	T = 0.01; // sample time
	ros::Rate loop_rate(1.0/T);


    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }




return 0;


	// main hybrid automata should be
	/* state 0: GoToGoal
	 * 		 1: FollowWall counter clockwise
	 * 		 2: FollowWwall clockwise
	 * 		 3: avoid obstacle
	 * 		 Exit state 0 if goal is close enough
	 */

	int state = 0;
	double Speed;
	std::vector<double> Goal(2,0);
	std::vector<double> Direction(2,0);
	std::vector<double> Motor_Speeds (2,0);
	Goal[0] = 1; Goal[1] = -1; // make it want to go 45 degrees to the right all the time


	switch (state) {
	case 0: // GoToGoal
		// switch conditions
		// if
		// ||x-x0|| = delta ( with some slack )
		// and <UGTG,UFWC> > 0
		// set d_tau = ||x-xG|| and set state = 2
		// if
		// ||x-x0|| = delta ( with some slack )
		// and <UGTG,UFWCC> > 0
		// set d_tau = ||x-xG|| and set state = 1
		// if close enough to goal, stop automata


		// set velocity to drive at ( could make it some function of the distance to the object, 1-e⁽...⁾ et.c.)
		Speed = R_Max_Speed;
		// normalize goal vector and multiple with velocity
		Direction[0] = Goal[0]/GetVectorLength(Goal)*Speed;
		Direction[1] = Goal[1]/GetVectorLength(Goal)*Speed;

		// Get motor Speeds
		Motor_Speeds = FollowVector(Direction);
		// Apply Motor Speeds




		break;
	case 1: // FollowWall Counter-Clockwise
		// switch conditions
		// if
		// ||x-xG|| < d_tau
		// and <UAO,UGTG> > 0
		// go back to state 0
		// if
		// ||x-x0|| < delta
		// go to state 3
		break;
	case 2:	// FollowWall Clockwise
		// switch conditions
		// if
		// ||x-xG|| < d_tau
		// and <UAO,UGTG> > 0
		// go back to state 0
		// if
		// ||x-x0|| < delta
		// go to state 3
		break;
	case 3:	// Avoid Obstacle (for safety)
		// switch conditions
		// if
		// ||x-x0|| = delta
		// and <UGTG,UFWCC> > 0
		// set d_tau = ||x-xG|| and go back to state 1
		// if
		// ||x-x0|| = delta
		// and <UGTG,UFWC> > 0
		// set d_tau = ||x-xG|| and go back to state 2
		break;
	}







/*
	std::vector <double> ObjDist (2,0);

	std::vector <double> u (2,0);
	u[0] = -0.1;
	u[1] = 0.1;
	std::cout << GetVectorLength(u) << std::endl;
	std::vector<double> Speeds (2,0);
	Speeds = FollowVector(u);
	std::cout << Speeds[0] << ":" << Speeds[1] << std::endl;

*/
	return 0;
}
