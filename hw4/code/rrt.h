#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "Graph.h"
#include "potentials.h"

struct treeNode{
    int vertex,parent;
};

class RRT{
        
    public:
        int n;
        double r, pgoal, e;
        bool valid_path;
        std::vector<int> x_bound, y_bound;
        Point qstart,qgoal;
        std::vector<Point> rrt_verts;
        std::vector<graphEdge> edges;
        std::vector<Point> midpoint;
        std::vector<treeNode> nodes;
        std::vector<int> path;

        //int gridH,gridW,top,bottom,left,right,buffer,scale_factor;
        std::vector<std::vector<Point>> obstacles;
        struct line{Point p1, p2;};

        // Constructor
        RRT(int n, double r, double e, double pgoal, std::vector<int> xbound, std::vector<int> ybound, Point qstart, Point qgoal);
        
        // Functions
        void retrieveObstacles(int num_obstacles,...);
        void createRRT();
        double dist(Point q1, Point q2);
        bool checkCollision(std::vector<Point> edge);
        bool intersect(line l1, line l2);
        int orientation(Point a, Point b, Point c);
        bool collinear(line c, Point p);

};
#endif