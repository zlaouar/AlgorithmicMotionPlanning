#ifndef PRM_H
#define PRM_H

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "Graph.h"
#include "potentials.h"

class Prm{
        
    public:
        int n;
        double r;
        double smooth_path_length;
        std::vector<int> x_bound, y_bound;
        Point qstart,qgoal;
        std::vector<Point> prm_verts;
        std::vector<graphEdge> edges;
        std::vector<Point> midpoint;

        //int gridH,gridW,top,bottom,left,right,buffer,scale_factor;
        std::vector<std::vector<Point>> obstacles;
        struct line{Point p1, p2;};

        // Constructor
        Prm(int n, double r, std::vector<int> xbound, std::vector<int> ybound, Point qstart, Point qgoal);
        
        // Functions
        bool valid(double x, double y);
        void retrieveObstacles(int num_obstacles,...);
        void createPrm();
        std::vector<graphNode> path_smooth(std::vector<graphNode> path_to_smooth, int num_its);
        double dist(Point q1, Point q2);
        bool checkCollision(std::vector<Point> edge);
        //double norm(Point delU);
        bool intersect(line l1, line l2);
        int orientation(Point a, Point b, Point c);
        bool collinear(line c, Point p);

};
#endif