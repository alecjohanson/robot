/*
 * HandController.h
 *
 *  Created on: Oct 5, 2013
 *      Author: robo
 */

#ifndef HANDCONTROLLER_H_
#define HANDCONTROLLER_H_

class HandController {
public:
	HandController();
	virtual ~HandController();
};

const double r0 = 5;
const double a0 = 5;
const double ka = 5;
const double kr = 5;

#endif /* HANDCONTROLLER_H_ */
