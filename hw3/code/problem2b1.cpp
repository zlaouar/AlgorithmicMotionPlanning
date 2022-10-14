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
    double alpha = 0.3;
    double zeta = 0.7;
    double dstar = 2;
    double eta = 2;
    vector<double> Qstar = {0.55,1};
    double epsilon = 0.25;
    Potentials planner(qstart, qgoal, alpha, zeta, epsilon, dstar, eta, Qstar);

    // 2 a)_______________________________________________________________________________
    // Retrieve obstacles
    int num_obstacles = 5;

    vector<Point> obstacle1{{1,1},{2,1},{2,5},{1,5}};
    vector<Point> obstacle2{{3,4},{4,4},{4,12},{3,12}};
    vector<Point> obstacle3{{3,12},{12,12},{12,13},{3,13}};
    vector<Point> obstacle4{{12,5},{13,5},{13,13},{12,13}};
    vector<Point> obstacle5{{6,5},{12,5},{12,6},{6,6}};
    planner.retrieveObstacles(num_obstacles, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5);//vector<Point> {{3,2},{5,2},{3,0},{5,0}},vector<Point> {{6,-2},{8,-2},{6,0},{8,0}});

    // Generate path for gradient descent algorithm
    planner.scale_factor = 6;
    planner.discretizeMap();
    vector<vector<Point>> field = planner.vectorField();

    
    // Write the vector field of the cspace to a csv file
    ofstream field_file;
    field_file.open ("field_2b1.csv");
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
    path_file.open ("path_2b1.csv");
    for(int i=0; i<path.size(); i++){
        path_file << path[i].x << "," << path[i].y <<"\n";
    }
    path_file.close();

    
    
    return 0;
}