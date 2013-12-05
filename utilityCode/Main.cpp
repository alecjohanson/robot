#include "NoiseReducer.h"
#include <stdio.h>
#include <iostream>

using namespace std;

int main()
{
    NoiseReducer nr;
    nr.initialize();
    int i=0;
    while (i<15)
    {
        i++;
        cerr << "i = " << i << "\n";
        nr.updateList(i);
        cerr << "Total in list = " << nr.totalInList << "\n";
        cerr << "Num Maxes in List = " << nr.numMaxesInList << "\n";
    }
    return 0;
}

