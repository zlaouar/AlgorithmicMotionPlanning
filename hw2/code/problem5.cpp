#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "cobstacle.h"

using namespace std;

#define PI 3.14159265

int main(){
    vector<vector<double>> obstacle{{1,2},{0,0},{0,2}}; // obstacle vertices
    vector<vector<double>> robot{{1,-1},{2,-1},{1,-3}}; // robot vertices
    double rot_size = PI/8; // rotation angle increment
    Cobstacle cspace;

    // 5 a)_______________________________________________________________________________
    // Generate cspace obstacle for robot with translation
    vector<vector<double>> cobstacle = cspace.getCobstacleTrans(obstacle,robot);
    
    // Write the vertices of the cspace obstacle to a csv file
    ofstream cobstacle_file;
    cobstacle_file.open ("c_obstacle.csv");
    for(int i=0; i<cobstacle.size(); i++){
        cobstacle_file << cobstacle[i][0] << "," << cobstacle[i][1] <<"\n";
    }
    cobstacle_file.close();

    // 5 b)_______________________________________________________________________________
    // Generate cspace obstacle for robot with translation and rotation
    vector<vector<double>> cobstacle_rot = cspace.getCobstacleRot(obstacle,robot,rot_size);
    
    // Write the vertices of the cspace obstacle to a csv file
    ofstream cobstacle_file_rot;
    cobstacle_file_rot.open ("c_obstacle_rot.csv");
    for(int i=0; i<cobstacle_rot.size(); i++){
        cobstacle_file_rot << cobstacle_rot[i][0] << "," << cobstacle_rot[i][1] <<"\n";
    }
    cobstacle_file_rot.close();
    
    return 0;
}