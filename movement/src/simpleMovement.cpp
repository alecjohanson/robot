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
static ros::Subscriber subscriber;
static ros::Subscriber odom_sub;

double const MAX_SPEED = 6;
double const MIN_SPEED = 1;
double const MAX_TURN_SPEED = MAX_SPEED;
double const MIN_TURN_SPEED = MIN_SPEED;
double ACCEL_COEFF = 6; //Increase for "smoother" transitions
double TURNING_COEFF = ACCEL_COEFF;

//Value to be set at beginning
double initialX = 0;
double initialY = 0;
double initialTheta = 0;
double previousTheta = 0;
double angleTraveled = 0;
bool turn;
int magnitude = 0;

ros::NodeHandle nh;

static const string odometryTopic = "differential_drive/Odometry";
void reset();
void setInitialOdometry(const differential_drive::Odometry &msg);
double calcWheelSpeed(double distanceFromEndPoint, bool turn);
void finalize();
void executeMovement(const differential_drive::Odometry &msg);

void move(const movement::Movement &msg)
{
    reset();
    turn = msg.turn;
    magnitude = msg.magnitude;
    //listen to encoder msgs
    nh.subscribe(odometryTopic,1,setInitialOdometry);
}

void reset()
{
    initialX = 0;
    initialY = 0;
    initialTheta = 0;
    previousTheta = 0;
    angleTraveled = 0;
    magnitude = 0;;
}

void setInitialOdometry(const differential_drive::Odometry &msg)
{
    initialX = msg.x;
    initialY = msg.y;
    initialTheta = msg.theta;
    odom_sub.shutdown();
    nh.subscribe(odometryTopic,1,executeMovement);
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
        if ((magnitude > 0 && angleTraveled > magnitude)
            || (magnitude < 0 && angleTraveled < magnitude))
        {
            finalize();
        }

        //Set wheel speeds to keep turning
        double angleToEndPoint = (angleTraveled < (magnitude - angleTraveled)) ? 
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
        double distance = hypot(initialX-msg.x,initialY-msg.y);
        if (distance > magnitude)
        {
            finalize();
        }
        //Calc wheel speeds
        double distanceToAnEndPoint = (distance < (magnitude - distance)) ? 
                                      distance : magnitude - distance;
        double wheelSpeed = calcWheelSpeed(distanceToAnEndPoint, false);        
        w1Speed = wheelSpeed;
        w2Speed = wheelSpeed;
    }

    spd.W1 = w1Speed;
    spd.W2 = w2Speed;
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


double calcWheelSpeed(double distanceFromEndPoint, bool turn)
{
    double max, min;
    double accel;
    if (turn)
    {
      max = MAX_TURN_SPEED;
      min = MIN_TURN_SPEED;
      accel = TURNING_COEFF;
    }
    else
    {
      max = MAX_SPEED;
      min = MAX_SPEED;
      accel = ACCEL_COEFF;
    }
    

    //Wheel speed is calculated on a simple piecewise function with constant accel and deccel
    if (distanceFromEndPoint > ACCEL_COEFF)
      return max;
    else
     return std::max(distanceFromEndPoint/accel*max, min);   
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");

    speed_pub = nh.advertise<differential_drive::Speed>("/motion/Speed",1);
    movement_completed_publisher = nh.advertise<movement::Movement>("/simpleMovement/moveCompleted",1);
    subscriber = nh.subscribe("/IRtest/movement",1,move);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
