#ifndef LOGIC_H_
#define LOGIC_H_

typedef struct{
	int nodeIdx;
	int direction;
} exploreNode_t;
void Move();
void goAhead();
int exists(double q);
void addNodeDirection();
void NewNode();
void Rotate(int angle);
void Advance();
void followWall(int wall);

#endif /* LOGIC_H_ */
