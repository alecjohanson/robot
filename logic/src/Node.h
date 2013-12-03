#ifndef NODE_H_
#define NODE_H_

#include <vector>

class Node {
public:
    Node();
    virtual ~Node();
    void setName(int name);
    void setType(int type[6]);
    void setOrientation(int orientation);
    void addParent (int parent);
    int* getNodeType();
    void setDistance(int distance);
    int getOrientation();
    double getDistance(int i);

private:
    int name;
    std::vector<int> parents;
    int type[6];
    int orientation;
    std::vector<double> distance;
};

#endif /* NODE_H_ */
