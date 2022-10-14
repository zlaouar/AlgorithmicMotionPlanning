#include "Bugs.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <cstdlib>

using namespace std;
/*
Bugs::Bugs(){
    height = 0;
}
*/
vector<vector<int>> generateMap(vector<vector<int>> obstacles, int w, int h, int scale_factor, int lowestx, int lowesty, int buffer){
    height = h;
    width = w;
    vector<vector<int>> map(h+1, vector<int> (w+1, 0));
    vector<vector<int>> WO(2, vector<int> (4, 0));
    for(int i=0; i<obstacles.size()+1; i++){
        if((i % 4 == 0 && i >= 3)){
            WO[0][0] = obstacles[i-4][0];
            WO[0][1] = obstacles[i-3][0];
            WO[0][2] = obstacles[i-2][0];
            WO[0][3] = obstacles[i-1][0];
            WO[1][0] = obstacles[i-4][1];
            WO[1][1] = obstacles[i-3][1];
            WO[1][2] = obstacles[i-2][1];
            WO[1][3] = obstacles[i-1][1];
            xlow = WO[0][0];
            xhigh = WO[0][0];
            ylow = WO[1][0];
            yhigh = WO[1][0];

            for(int m=0; m < WO[0].size(); m++){
                if(WO[0][m] < xlow){
                    xlow = WO[0][m];
                }
                else if(WO[0][m] > xhigh){
                    xhigh = WO[0][m];
                }

                if(WO[1][m] < ylow){
                    ylow = WO[1][m];
                }
                else if(WO[1][m] > yhigh){
                    yhigh = WO[1][m];
                }
            }
            cout << "xlow: "  << xlow << " xhigh: "  << xhigh << " ylow: "  << ylow << " yhigh: "  << yhigh << endl;
            
            for(int n = xlow; n < xhigh+1; n++){
                for(int p = ylow; p < yhigh+1; p++){
                    //cout << height - (p +lowesty) << endl;
                    map[height - (p +abs(lowesty) + buffer)][n+abs(lowestx)+buffer] = 1;
                }
            }
        }
    }
    return map;
}