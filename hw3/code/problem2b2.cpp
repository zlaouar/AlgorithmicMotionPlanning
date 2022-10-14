#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "potentials.h"

using namespace std;

#define PI 3.14159265

int main(){
    Point qstart{0,0};
    Point qgoal{35,0};
    double alpha = 0.3;
    double zeta = 0.1;
    double dstar = 10;
    double eta = 5;
    vector<double> Qstar = {1,1};
    double epsilon = 0.25;
    Potentials planner(qstart, qgoal, alpha, zeta, epsilon, dstar, eta, Qstar);

    // 2 a)_______________________________________________________________________________
    // Retrieve obstacles
    int num_obstacles = 9;

    vector<Point>obstacle1{{-6,-6},{25,-6},{25,-5},{-6,-5}};
    vector<Point>obstacle2{{-6,5},{30,5},{30,6},{-6,6}};
    vector<Point>obstacle3{{-6,-5},{-5,-5},{-5,5},{-6,5}};
    vector<Point>obstacle4{{4,-5},{5,-5},{5,1},{4,1}};
    vector<Point>obstacle5{{9,0},{10,0},{10,5},{9,5}};
    vector<Point> obstacle6{{14,-5},{15,-5},{15,1},{14,1}};
    vector<Point> obstacle7{{19,0},{20,0},{20,5},{19,5}};
    vector<Point> obstacle8{{24,-5},{25,-5},{25,1},{24,1}};
    vector<Point> obstacle9{{29,0},{30,0},{30,5},{29,5}};
    planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9);

    // Generate path for gradient descent algorithm
    planner.scale_factor = 4;
    planner.discretizeMap();
    vector<vector<Point>> field = planner.vectorField();

    
    // Write the vector field of the cspace to a csv file
    ofstream field_file;
    field_file.open ("field_2b2.csv");
    field_file << planner.top << "," << planner.bottom << "," << planner.left << "," << planner.buffer << "," << planner.scale_factor << endl;
    for(int i=0; i<field.size(); i++){
        for(int j=0; j<field[0].size(); j++){
            field_file <<field[i][j].x << "," <<field[i][j].y << ",";
        }
        field_file << endl;
    }
    field_file.close();

    
    // Write the path of the robot to a csv file
    vector<Point> path = planner.gradientDescent();
    ofstream path_file;
    path_file.open ("path_2b2.csv");
    for(int i=0; i<path.size(); i++){
        path_file << path[i].x << "," << path[i].y <<"\n";
    }
    path_file.close();

    
    
    return 0;
}