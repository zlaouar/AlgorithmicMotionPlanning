#ifndef POTENTIALS_H
#define POTENTIALS_H

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

struct Point{
    double x, y;
    Point operator-(const Point& rhs){return {x-rhs.x,y-rhs.y};}
    Point operator+(const Point& rhs){return {x+rhs.x,y+rhs.y};}
    Point operator/(double k){return {x/k,y/k};}
    Point operator*(double k){return {x*k,y*k};}
};

class Potentials{
    private:
        Point qstart,qgoal,q;
        double alpha,zeta,epsilon,dstar,eta;
        std::vector<double> Qstar;
        
    public:
        int gridH,gridW,top,bottom,left,right,buffer,scale_factor;
        std::vector<std::vector<int>> map;
        std::vector<std::vector<Point>> obstacles;
        Potentials(Point qstart, Point qgoal, double alpha, double zeta, double epsilon, double dstar, double eta, std::vector<double> Qstar);
        // Problem 2: Attractive Repulsive Functions
        void retrieveObstacles(int num_obstacles,...);
        std::vector<Point> gradientDescent();
        std::vector<std::vector<Point>> vectorField();
        double dist(Point q, Point qgoal);
        Point attrPotential(Point cstate);
        double norm(Point delU);

        void discretizeMap();
        Point repPotential(Point cspace);
        std::vector<std::vector<int>> brushFire();
        bool borderPixel(int i, int j, int num, bool eight_point);
        std::vector<bool> validDirections(int i, int j);

        // Wave Front Functions
        Potentials(Point qstart, Point qgoal);
        std::vector<Point> waveFront();

};
#endif