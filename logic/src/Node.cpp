#include <iostream>
#include "Node.h"

Node::Node() {
    // TODO Auto-generated constructor stub
}

void Node::setName(int name){this->name = name;}

void Node::setType(int type[6]){
	for(int i=0; i<6; ++i)
		this->type[i] = type[i];
}

void Node::setOrientation(int orientation){this->orientation = orientation;}

void Node::addParent(int parent){this->parents.push_back(parent);}

int* Node::getNodeType(){return this->type;}

Node::~Node() {
    // TODO Auto-generated destructor stub
}
