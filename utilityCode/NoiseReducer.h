#ifndef NOISEREDUCER_H
#define NOISEREDUCER_H
#endif

struct Node
{
  double value;
  Node *child;
  bool null;
};
//Used internally
  static Node *topNode; //Added most recent 
  static Node *bottomNode; //Oldest, to be removed next
  static int numNodesInList;


struct NoiseReducer
{
  //Functions
    void initialize();
    void updateList(double val);
    void getAverage();

  //Parameters //Set to default values in NoiseReducer.cpp
    int NUMBER_OF_NODES_TO_CHECK;
    double PERCENT_OF_MAXES_NEEDED;
    double THRESHOLD;

  //Use these for analyzing state of the list
    int numMaxesInList;
    double totalInList;
};



