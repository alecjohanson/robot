#include <iostream>
#include <ros/ros.h>
#include <sharps/Distance.h>
#include <differential_drive/Speed.h>
#include "IRtest.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <iomanip>
using namespace std;

static ros::Publisher speed_pub;
static ros::Publisher movement_completed_publisher;
static ros::Subscriber subscriber;
static ros::Subscriber odom_sub;

double const MAX_SPEED = 6;
double const MIN_SPEED = 1;
double const MAX_TURN_SPEED = MAX_SPEED;
double const MIN_TURN_SPEED = MIN_SPEED;
double ACCEL_COEFF = 6; //Increase for "smoother" transitions
double TURNING_COEFF = ACCEL_COEFF;

//Value to be set at beginning
float initialX = 0;
float initialY = 0;
float initialTheta = 0;
float previousTheta = 0;
float angleTraveledi = 0;
bool turn;
int magnitude = 0;;

private const string odometryTopic = "differentialDrive/Odometry";

void move(const movement::Movement &msg)
{
    reset();
    turn = msg.turn;
    magnitude = msg.magnitude;
    //listen to encoder msgs
    odom_sub.subscribe(odometryTopic,1,setInitialOdometry);
}

void reset()
{
    initialX = 0;
    initialY = 0;
    initialTheta = 0;
    previousTheta = 0;
    angleTraveledi = 0;
    magnitude = 0;;
}

void setInitialOdometry(const differentialDrive::Odometry &msg)
{
    this.initialX = msg.x;
    this.initialY = msg.y;
    this.initialTheta = msg.theta;
    odom_sub.shutdown();
    odom_sub.subscribe(odometryTopic,1,executeMovement);
}

void executeMovement(const differentalDrive::Odometry &msg)
{
    Speed spd;
    double w1Speed;
    double w2Speed;

    if (turn)
    {
        //This is done this way so > 2pi can be achieved if weirdly desired
        angleTraveled += msg.theta - previousTheta;
        previousTheta = msg.theta;
        if ((magnitude > 0 && angleTraveled > magnitude)
            || (magnitude < 0 && angleTraveled < magnitude))
        {
            finalize();
        }

        //Set wheel speeds to keep turning
        float angleToEndPoint = (angleTraveled < (magnitude - angleTraveled)) ? 
                                      angleTraveled : magnitude - angleTraveled;
        double wheelSpeed = calcWheelSpeed(angleToEndPoint, true);

        if (angleTraveled > magnitude)
        {
            w1Speed = wheelSpeed;
            w2Speed = -wheelSpeed;
        }
        else
        {
            w1Speed = -wheelSpeed;
            w2Speed = wheelSpeed;
        }
    }
    else //Move strait
    {
        //Check Distance traveled
        //TODO:: maybe put in checks to make sure theta stays the same
        float distance = distanceBetween(initialX, msg.x, intitialY, msg.y);
        if (distance > magnitude)
        {
            finalize();
        }
        //Calc wheel speeds
        float distanceToAnEndPoint = (distance < (magnitude - distance)) ? 
                                      distance : magnitude - distance;
        double wheelSpeed = calcWheelSpeed(distance, false);        
        w1Speed = wheelSpeed;
        w2Speed = wheelSpeed;
    }

    spd.w1 = w1Speed;
    spd.w2 = w2Speed;
    spd.header.stamp = ros::Time::now();

    //Completed the movement
    movement_completed_publisher.publish(msg);
}

void finalize()
{
     //Stop subscribing and wasting the calculation saying we are finished
    odom_sub.shutdown();
    //TODO::Advertise we are done?
}


double calcWheelSpeed(float distanceFromEndPoint, bool turn)
{
    float max, min;
    double accel;
    if (turn)
    {
      max = MAX_TURN_SPEED;
      min = MIN_TURN_SPEED;
      accl = TURNING_COEFF;
    }
    else
    {
      max = MAX_SPEED;
      min = MAX_SPEED;
      accl = ACCEL_COEFF;
    }
    

    //Wheel speed is calculated on a simple piecewise function with constant accel and deccel
    if (distanceFromEndPoint > ACCEL_COEFF)
      return max;
    else
     return max(distanceFromEndPoint/accl*max, min);   
}

float distanceBetween(x1,x2,y1,y2)
{
        float tempX = x1 - x2; 
        float tempY = y1 - y2;

        float tempX = tempX * tempX;
        float tempY = tempY * tempY;

        float distance = tempX + tempY;
        return sqrt(distance);
}


int main(int argc, const char* argv[])
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    speed_pub = nh.advertise<Speed>("/motion/Speed",1);
    movement_completed_publisher = nh.advertise<Movement>("/simpleMovement/moveCompleted");
    subscriber = nh.subscribe("/IRtest/movement",1,move);

    ros::Rate looprate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
