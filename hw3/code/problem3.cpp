#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "potentials.h"

using namespace std;

#define PI 3.14159265

int main(){
    Point qstart{0,0};
    Point qgoal{10,10};
    
    Potentials planner(qstart, qgoal);

    // 3 a) Map 1_______________________________________________________________________________
    // Retrieve obstacles
    int num_obstacles = 5;

    vector<Point> obstacle1{{1,1},{2,1},{2,5},{1,5}};
    vector<Point> obstacle2{{3,4},{4,4},{4,12},{3,12}};
    vector<Point> obstacle3{{3,12},{12,12},{12,13},{3,13}};
    vector<Point> obstacle4{{12,5},{13,5},{13,13},{12,13}};
    vector<Point> obstacle5{{6,5},{12,5},{12,6},{6,6}};
    planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5);

    // Generate path for gradient descent algorithm
    planner.scale_factor = 2; // Set pixel width to 0.25
    planner.discretizeMap();// Create discretized map with obstacles as 1s

    // Write the path of the robot to a csv file
    vector<Point> path = planner.waveFront();
    ofstream path_file;
    path_file.open ("path_3_map1.csv");
    path_file << planner.top << "," << planner.bottom << "," << planner.left << "," << planner.buffer << "," << planner.scale_factor << endl;
    for(int i=0; i<path.size(); i++){
        path_file << path[i].x << "," << path[i].y <<"\n";
    }
    path_file.close();

    
    // 3 a) Map 2___________________________________________________________________________________
    qstart = {0,0};
    qgoal = {35,0};
    
    Potentials planner1(qstart, qgoal);

    // Retrieve obstacles
    num_obstacles = 9;

    obstacle1 = {{-6,-6},{25,-6},{25,-5},{-6,-5}};
    obstacle2 = {{-6,5},{30,5},{30,6},{-6,6}};
    obstacle3 = {{-6,-5},{-5,-5},{-5,5},{-6,5}};
    obstacle4 = {{4,-5},{5,-5},{5,1},{4,1}};
    obstacle5 = {{9,0},{10,0},{10,5},{9,5}};
    vector<Point> obstacle6{{14,-5},{15,-5},{15,1},{14,1}};
    vector<Point> obstacle7{{19,0},{20,0},{20,5},{19,5}};
    vector<Point> obstacle8{{24,-5},{25,-5},{25,1},{24,1}};
    vector<Point> obstacle9{{29,0},{30,0},{30,5},{29,5}};
    planner1.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9);

    // Generate path for gradient descent algorithm
    planner1.scale_factor = 2; // Set pixel width to 0.25
    planner1.discretizeMap();// Create discretized map with obstacles as 1s

    // Write the path of the robot to a csv file
    vector<Point> path1 = planner1.waveFront();
    ofstream path_file1;
    path_file1.open ("path_3_map2.csv");
    path_file1 << planner1.top << "," << planner1.bottom << "," << planner1.left << "," << planner1.buffer << "," << planner1.scale_factor << endl;
    for(int i=0; i<path1.size(); i++){
        path_file1 << path1[i].x << "," << path1[i].y <<"\n";
    }
    path_file1.close();
    return 0;
}