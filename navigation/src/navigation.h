#ifndef NAVIGATION_H_
#define NAVIGATION_H_

double GetVectorLength(std::vector<double>& vector);

/* Creates a vector to the object to be avoided, (virtual object)
 * Calculates this by just taking the closest object right now.
 * UPGRADE: should take all information about  objects into account,
 * weighing each one depending on distance and also which direction the robot is
 * heading.
 *
 * Takes input measurements in meters
 * Vectors are defined with x in the robots direction, y = x*Rotate(pi/2)
 * and origo in the robots centre of rotation
 */
std::vector<double> GetObjVector();

/* Gets a vecctor to goal and return appropriate wheel speeds in angular velocities (Right wheel, Left Wheel)
 * length of goal vector is robot speed.
 */
std::vector <double> FollowVector (std::vector<double> U);

// get vector to follow if you want to avoid obstacles
std::vector <double> GetAvoidObjectVector ();



#endif /* NAVIGATION_H_ */
