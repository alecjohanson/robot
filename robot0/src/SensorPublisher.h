#ifndef SENSORPUBLISHER_H_
#define SENSORPUBLISHER_H_

// *******************
// Regression constants
// *******************

// Front Right Sensor
const double alpha_1 = 2666;                 // constant for calibration
const double beta_1 = -14;                   // constant for calibration

// Rear Right Sensor
const double alpha_2 = 2666;                 // constant for calibration
const double beta_2 = -14;                   // constant for calibration

// Front Left Sensor
const double alpha_3 = 2666;                 // constant for calibration
const double beta_3 = -14;                   // constant for calibration

// Rear Left Sensor
const double alpha_4 = 2666;                 // constant for calibration
const double beta_4 = -14;                   // constant for calibration

// Rear Sensor
const double alpha_5 = 2666;                 // constant for calibration
const double beta_5 = -14;                   // constant for calibration

// Front Sensor
const double alpha_6 = 2666;                 // constant for calibration
const double beta_6 = -14;                   // constant for calibration

// *******************
// Calibration Dist.
// *******************

// Distance to remove from measurement
const double Sharp2Wheel_FrontRightD = 6.0;  // distance from sharp ch8 to wheel
const double Sharp2Wheel_RearRightD = 6.2;   // distance from sharp ch4 to wheel

const double Sharp2Wheel_FrontLeftD = 4.8;   // distance from sharp ch7 to wheel
const double Sharp2Wheel_RearLeftD = 6.5;    // distance from sharp ch5 to wheel

const double Sharp2Edge_RearD = 2.0;         // distance from sharp ch2 to edge
const double Sharp2Edge_FrontD = 0.0;        // sharp ch3 to edge

#endif /* SENSORPUBLISHER_H_ */
