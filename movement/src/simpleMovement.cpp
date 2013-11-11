#include <iostream>
#include <ros/ros.h>
#include <sharps/Distance.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Odometry.h>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <movement/Movement.h>
using namespace std;

static ros::Publisher speed_pub;
static ros::Publisher movement_completed_publisher;
static ros::Subscriber move_sub;
static ros::Subscriber odom_sub;

double const MAX_SPEED = (0.5/0.05);
double const MIN_SPEED = (0.02/0.05);
double const MAX_TURN_SPEED = (MAX_SPEED*0.8);
double const MIN_TURN_SPEED = (MIN_SPEED);
double const ACCEL_RANGE_LIN = 0.2;
double const ACCEL_RANGE_ROT = (M_PI*0.5);

//Value to be set at beginning
double initialX = 0;
double initialY = 0;
double previousTheta = 0;
double angleTraveled = 0;
bool turn;
double magnitude = 0;
volatile enum state_t {
	STBY, INIT, MOVING
} state;

void reset();
void odometryHandler(const differential_drive::Odometry &msg);
double calcWheelSpeed(double distanceFromEndPoint, bool turn);
void finalize();
void executeMovement(const differential_drive::Odometry &msg);

void move(const movement::Movement &msg)
{
    reset();
    turn = msg.turn;
    magnitude = msg.magnitude;
    state=INIT;
}

void reset()
{
    initialX = 0;
    initialY = 0;
    previousTheta = 0;
    angleTraveled = 0;
    magnitude = 0;
}

void odometryHandler(const differential_drive::Odometry &msg)
{
	switch(state) {
	case STBY: return;
	case INIT:
		initialX = msg.x;
		initialY = msg.y;
		previousTheta = msg.theta;
		state=MOVING;
		break;
	case MOVING:
		executeMovement(msg);
		break;
	}
}

void executeMovement(const differential_drive::Odometry &msg)
{
    differential_drive::Speed spd;
    double w1Speed;
    double w2Speed;

    if (turn)
    {
        //This is done this way so > 2pi can be achieved if weirdly desired
        angleTraveled += msg.theta - previousTheta;
        previousTheta = msg.theta;
	double wheelSpeed;
	double angleToEndPoint=0.0;
        if ((magnitude > 0 && angleTraveled > magnitude)
            || (magnitude < 0 && angleTraveled < magnitude))
        {
            finalize();
		wheelSpeed=0.0;
        } else {
		//Set wheel speeds to keep turning
		angleToEndPoint = std::min(abs(angleTraveled),abs(magnitude - angleTraveled));
		wheelSpeed = calcWheelSpeed(angleToEndPoint, true);
	}

        if (angleTraveled > magnitude)
        {
            w1Speed = -wheelSpeed;
            w2Speed = wheelSpeed;
        }
        else
        {
            w1Speed = wheelSpeed;
            w2Speed = -wheelSpeed;
        }
	cerr<<angleTraveled*180./M_PI<<' '<<angleToEndPoint*180./M_PI<<' '<<w1Speed<<endl;
    }
    else //Move strait
    {
        //Check Distance traveled
        //TODO:: maybe put in checks to make sure theta stays the same
        double distance = hypot(initialX-msg.x,initialY-msg.y);
        if (distance > magnitude)
        {
            finalize();
		w1Speed=0.0;
        } else {
		//Calc wheel speeds
		double distanceToAnEndPoint = (distance < (magnitude - distance)) ? 
		                              distance : magnitude - distance;
		w1Speed = calcWheelSpeed(distanceToAnEndPoint, false);
	}
	cerr<<distance<<' '<<w1Speed<<endl;
	w2Speed=w1Speed;
    }

    spd.W1 = w1Speed;
    spd.W2 = w2Speed;
    spd.header.stamp = ros::Time::now();
    speed_pub.publish(spd);

}

void finalize()
{
     //Stop subscribing and wasting the calculation saying we are finished
    state=STBY;
    //TODO::Advertise we are done?
    //Completed the movement
    //movement_completed_publisher.publish(msg);
}


double calcWheelSpeed(double distanceFromEndPoint, bool turn)
{
/*
double const MAX_SPEED = (1.0/0.05);
double const MIN_SPEED = (0.1/0.05);
double const MAX_TURN_SPEED = (MAX_SPEED*0.5);
double const MIN_TURN_SPEED = (MIN_SPEED*0.8);
double const ACCEL_RANGE_LIN 0.1;
double const ACCEL_RANGE_ROT (M_PI*0.25);
*/
    if (turn)
    {
	if(distanceFromEndPoint>ACCEL_RANGE_ROT) return MAX_TURN_SPEED;
	else return MIN_TURN_SPEED+(distanceFromEndPoint/ACCEL_RANGE_ROT)*(MAX_TURN_SPEED-MIN_TURN_SPEED);
    }
    else
    {
	if(distanceFromEndPoint>ACCEL_RANGE_LIN) return MAX_SPEED;
	else return MIN_SPEED+(distanceFromEndPoint/ACCEL_RANGE_LIN)*(MAX_SPEED-MIN_SPEED);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    speed_pub = nh.advertise<differential_drive::Speed>("/motion/Speed",1);
    movement_completed_publisher = nh.advertise<movement::Movement>("/simpleMovement/moveCompleted",1);
    move_sub=nh.subscribe("/IRtest/movement",1,move);
    odom_sub=nh.subscribe("/motion/Odometry",1,odometryHandler);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
