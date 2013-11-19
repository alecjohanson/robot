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
#include "NoiseReducer.h"

void NoiseReducer::initialize()
{
  topNode = new Node;
  bottomNode = new Node;
  topNode->null = true;
  bottomNode->null = true;
  //Default Parameters
  NUMBER_OF_NODES_TO_CHECK = 4;
  PERCENT_OF_MAXES_NEEDED = .9;
  THRESHOLD = 10;
  //Initializing vals
  numMaxesInList = 0;
  totalInList=0;
}

void NoiseReducer::updateList(double val)
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

double getAverage()
{
  return totalInList/numNodesInList;
}

