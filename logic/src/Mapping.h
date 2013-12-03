#ifndef MAPPING_H_
#define MAPPING_H_

typedef struct{
	int nodeIdx;
	int direction;
} exploreNode_t;
void Move();
void goAhead();
void addNodeDirection();
void NewNode();
void Rotate(int angle);
void Advance();
void followWall(int wall);
int normalizeAngle(int angle);

#endif /* MAPPING_H_ */
