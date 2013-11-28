//MapNode.h



#ifndef MAPNODE_H_
#define MAPNODE_H_

#include "Vec2D.h"
#include <vector>

class MapNode{
public:
	MapNode();
	MapNode(double x, double y, std::vector<int> dir,int id){
		setPos(x,y);
		setDir(dir);
		setConnection(NULL,-1);
		setDistance(0,-1);
		setNext(NULL);
		setPrev(NULL);
		setsNext(NULL);
		setsPrev(NULL);
		setID(id);
	}
	MapNode(Vec2D<double> p, std::vector<int> dir,int id){
		setPos(p.x,p.y);
		setDir(dir);
		setConnection(NULL,-1);
		setDistance(0,-1);
		setNext(NULL);
		setPrev(NULL);
		setsNext(NULL);
		setsPrev(NULL);
		setID(id);
	}
	~MapNode(){
		delete[] n_connection;
		delete n_next;
		delete n_prev;
		delete s_next;
		delete s_prev;
	}

	Vec2D<double> getPos(){return n_pos;}
	int getExplored(int i) {return n_dir[i];}
	MapNode * getConnection(int dir){return n_connection[dir];}
	MapNode * getNext(){return n_next;}
	MapNode * getPrev(){return n_prev;}
	MapNode * getsNext(){return s_next;}
	MapNode * getsPrev(){return s_prev;}
	double getDistance(int dir){return n_distance[dir];}
	void setID(int id) {n_id = id;}
	int  getID() {return n_id;}
	void setDir(std::vector<int> dir) {n_dir = dir;}
	void setPos(Vec2D<double>  pos) {n_pos = pos;}
	void setPos(double x, double y) {n_pos.x = x; n_pos.y = y;}
	void setNext(MapNode * next){n_next = next;}
	void setPrev(MapNode * prev){n_prev = prev;}
	void setsPrev(MapNode * prev){s_prev = prev;}
	void setsNext(MapNode * next){s_next = next;}
	void setTmp(int tmp){n_tmp = tmp;}
	int  getTmp() {return n_tmp;}
	void setCost(double cost){n_cost = cost;}
	double  getCost() {return n_cost;}
	void setConnection(MapNode * connection,int dir){
		if (dir == -1) {
			n_connection[0] = connection;
			n_connection[1] = connection;
			n_connection[2] = connection;
			n_connection[3] = connection;
		} else {
			n_connection[dir] = connection;
			
			Vec2D<double> dist = (connection->getPos()-getPos());

			setDistance(pow(pow(dist.x,2)+pow(dist.y,2),0.5),dir);

			if (connection->isExplored()) setExplored(dir);

		}
	}
	void setDistance(double distance,int dir){
		if (dir == -1) {
			n_distance[0] = distance;
			n_distance[1] = distance;
			n_distance[2] = distance;
			n_distance[3] = distance;
		} else {n_distance[dir] = distance;}
	}
	bool isExplored() {
		if (n_dir[0] != 1 && n_dir[1] != 1 && n_dir[2] != 1 \
			&& n_dir[3] != 1) return true;
		else return false;
	}
	


	void setExplored(int i) {
		if (n_dir[i] == 1) n_dir[i] = 2;			
	}
	


private:
	Vec2D<double> n_pos;
	std::vector<int> n_dir; // available directions
	MapNode * n_connection[4];
	MapNode * n_next;
	MapNode * n_prev;
	MapNode * s_next; // for searching
	MapNode * s_prev;
	double n_distance[4];
	int n_id;
	int n_tmp;
	double n_cost;
};


#endif /* MAPNODE_H_ */

