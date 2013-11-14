//---------------------------------------------
// A template of code that was created to easily
//  plug in and implement as a noise reducer
//
//  If you get a SegFault make sure you initialized
//  it first!
//
//  created by Alec
//-----------------------------------------------


#include <cmath>
#include <iostream>
#include <stdio.h>

using namespace std;

//--------------------------------
//   CONSTANTS
//--------------------------------
static const int NUMBER_OF_NODES_TO_CHECK = 4;
static const double PERCENT_OF_MAXES_NEEDED = .9;
static const double THRESHOLD = 10;
//////////////////////////////////////////
//Use these for analyzing state of the list
int numMaxesInList=0;
static double totalInList;
/////////////////////////////////////

void updateList(double val);
void initialize();

struct Node
{
  double value;
  Node *child;
  bool null;
};

static Node *topNode; //Added most recent 
static Node *bottomNode; //Oldest, to be removed next
static int numNodesInList;

int main(int arc, char** argv)
{
      initialize();
      int i =0;
      while(i<15)
      {
        i++;
        cerr << "i = " << i << "\n";
        updateList(i);
        cerr << "Total in list = " << totalInList << "\n";
        cerr << "Num Maxes in List = " << numMaxesInList << "\n";
      }
      return 0;
}


void initialize()
{
  topNode = new Node;
  bottomNode = new Node;
  topNode->null = true;
  bottomNode->null = true;
}

void updateList(double val)
{
  totalInList += val;
  if (val > THRESHOLD)
     numMaxesInList++;
  Node *newNode;
  newNode = new Node;
  newNode->value = val;
  if (topNode->null)  //Creating a new list
  {
    topNode = newNode;
    bottomNode = newNode;
    return;
  }
  //We always want to add this to the front/
  Node temp;
  topNode->child = newNode;
  topNode = newNode;

  if(numNodesInList <= NUMBER_OF_NODES_TO_CHECK) //We are not going to delete the last because the list isn't ful
  {
    numNodesInList++;
    return;
  }
  
  if(bottomNode->value > THRESHOLD)
    numMaxesInList--;

  totalInList -= bottomNode->value;

  //Remove bottom node
  bottomNode = bottomNode->child;
}
