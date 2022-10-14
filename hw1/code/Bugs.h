#ifndef Bugs_H
#define Bugs_H
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <fstream>


using namespace std;
class Bugs{

        //vector<vector<int>> map(h+1, vector<int> (w+1, 0));
        //vector<vector<int>> WO(2, vector<int> (4, 0));
        int const height;
        int const width;
        int xlow;
        int xhigh;
        int ylow;
        int yhigh;
    public:
        //Bugs();
        vector<vector<int>> generateMap(vector<vector<int>>, int, int, int, int, int, int);

};
#endif