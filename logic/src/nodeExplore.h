//
#ifndef NODEEXPLORE_H_
#define NODEEXPLORE_H_
class nodeExplore {
public:
	int name;
	double disposition[4][3];
	double position[2];
	//nodeExplore();
	//~nodeExplore();
	void init();


};

void nodeExplore::init(){
	name=0;
	for(int i=0;i<4;i++){
		disposition[i][0]=-1;
	}
}

#endif /*NODEEXPLORE_H_*/
