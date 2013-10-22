#ifndef IRTEST_H_
#define IRTEST_H_

const double alpha_1 = 2666; 		// constant for calibration
const double beta_1 = -14; 			// constant for calibration
const double alpha_2 = 2666; 		// constant for calibration
const double beta_2 = -14; 			// constant for calibration
const double Sharp2Wheel_FrontRightD = 6.0;	// distance from sharp ch8 to wheel
const double Sharp2Wheel_RearRightD = 6.5;	// distance from sharp ch4 to wheel
const double IR_rightD = 16.2;		// distance between ch4 and ch8.
const double K_forward = 0.5; 			// Control coefficient --> ask GB.
const double Wheel2Wall_D = 7; 		// distance from the wheel to a wall.
const double k_theta = 7;			// Control coefficient --> Ask GB.
const double RobotSpeed = 6;		// Constant speed of the robot.
const double MaxFowardAngle = 0.2;	// Max angle where the robot is allowed to move forward in radians.

#endif /* IRTEST_H_ */
